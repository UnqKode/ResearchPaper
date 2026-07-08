from traci_compat import traci, USING_LIBSUMO
tc = traci.constants
from RSU.rsu import RSU
from collections import deque
import zlib

# ---------------------------------------------------------------------------
# TraCI subscription variable sets (fetched in bulk each step).
# ---------------------------------------------------------------------------

# Per-edge variables subscribed once at startup for every tracked edge.
# tc.LAST_STEP_VEHICLE_ID_LIST  (18)  – IDs of vehicles on the edge last step
# tc.LAST_STEP_OCCUPANCY        (19)  – edge occupancy 0-100 %
# tc.LAST_STEP_VEHICLE_HALTING_NUMBER (20) – number of stopped vehicles
_EDGE_VARS = (
    tc.LAST_STEP_VEHICLE_ID_LIST,
    tc.LAST_STEP_OCCUPANCY,
    tc.LAST_STEP_VEHICLE_HALTING_NUMBER,
    tc.VAR_FUELCONSUMPTION,   # edge-level total fuel this step (mg/s sum)
    tc.LAST_STEP_MEAN_SPEED,  # for speed_ratio in corridor gate
)

# --- Vehicle sampling (Fix B) -------------------------------------------------
# Per-vehicle subscriptions are the dominant socket cost each step. We subscribe
# only a deterministic 1/sample_mod fraction of ordinary vehicles (plus every
# ego), keeping per-vehicle fuel/speed/halt tracking for that sample. The
# sampling is edge-blind (hash of the vehicle ID), so a given vehicle is either
# sampled everywhere or nowhere for the whole run.
_DEFAULT_SAMPLE_MOD = 5


def _is_sampled(vid: str, sample_mod: int) -> bool:
    """Deterministic vehicle sampling by ID hash. Edge-blind."""
    return (zlib.crc32(vid.encode()) % sample_mod) == 0


def _is_ego(vid: str) -> bool:
    return vid.startswith("ego_")

# Per-vehicle variables subscribed when a vehicle first enters a tracked edge.
# Results are returned by getAllSubscriptionResults() in bulk each step.
# tc.VAR_SPEED          (64)  – current speed (m/s)
# tc.VAR_WAITING_TIME  (122)  – accumulated waiting time (s)
# tc.VAR_FUELCONSUMPTION(101) – instantaneous fuel rate (mg/s)
# tc.VAR_CO2EMISSION    (96)  – instantaneous CO2 rate (mg/s)
_VEH_VARS = (
    tc.VAR_SPEED,
    tc.VAR_WAITING_TIME,
    tc.VAR_FUELCONSUMPTION,
    tc.VAR_CO2EMISSION,
)


class RSUManager:
    """
    Manages one RSU per intersection, collects per-edge traffic data from
    TraCI each step, and feeds fuel observations to EdgeCostCalculator.

    PERFORMANCE — TraCI subscription model
    ---------------------------------------
    Instead of issuing one TraCI call per vehicle per step (O(E × V) socket
    round-trips), this class uses the TraCI subscription API:

      • Edge subscriptions (set once via subscribe_edges() after traci.start):
        - LAST_STEP_VEHICLE_ID_LIST  → replaces getLastStepVehicleIDs per edge
        - LAST_STEP_OCCUPANCY        → replaces getLastStepOccupancy per edge
        - LAST_STEP_VEHICLE_HALTING_NUMBER → replaces getLastStepHaltingNumber

      • Vehicle subscriptions (set per vehicle on first appearance):
        - VAR_SPEED / VAR_WAITING_TIME / VAR_FUELCONSUMPTION / VAR_CO2EMISSION

    In step(), TWO bulk calls replace the previous ~24 000 individual calls:
        edge_results = traci.edge.getAllSubscriptionResults()
        veh_results  = traci.vehicle.getAllSubscriptionResults()

    NOTE: vehicle subscription results lag by one step (data available from
    the step AFTER subscribe() is called). For vehicles that stay on an edge
    for many steps (typical), one skipped step is negligible. The first step
    on an edge accumulates no data; accumulation begins on the second step.

    RSU COVERAGE POLICY (Q1):
      Every non-internal edge is assigned to exactly one RSU:
        * Edges whose to_node is an intersection  -> RSU at to_node (preferred).
        * Edges whose to_node is a dead_end       -> RSU at from_node instead,
          so no peripheral edge is orphaned and falls back to static weight forever.
      A startup log reports the fraction of edges covered.

    Data semantics for the per-departure data point:
      - vehicle_count / avg_speed / waiting_time / stop_and_go_freq /
        fuel_consumption / co2_emissions are TRIP AGGREGATES over the vehicles
        that just finished traversing the edge.
      - queue_length / occupancy are an INSTANTANEOUS SNAPSHOT of the edge at
        the moment those vehicles departed (they describe the road right now,
        not the departed vehicles).
    """

    def __init__(self, intersections, edges, window_size=60,
                 sample_mod=_DEFAULT_SAMPLE_MOD):
        # Window size for RSU rolling statistics: number of vehicle-departure
        # EVENTS kept per edge (not simulation steps). At typical Monaco traffic
        # rates (~2-3 veh/min on busy edges), 60 events ≈ 20-30 min of history --
        # enough to smooth short-term noise while reacting within ~30 min to a new
        # degradation event (e.g., EU0 grade mode activation).
        # 240 events was too large: at 2.6 veh/min the 1800s EU4 warmup contributes
        # 78 entries that dilute the EU0 signal for >90 min after grade activation,
        # collapsing F from 0.165 (theoretical) to ~0.10 (observed).
        # With 60 events, warmup entries flush out within ~23 min of grade start,
        # giving F≈0.165 at the 45-min ego-injection window.  The value is
        # stored here so initialize_from_network can pass it through to each RSU.
        self.window_size = window_size
        # Fix B: only 1/sample_mod of ordinary vehicles (plus every ego) are
        # subscribed and tracked per-vehicle. Trades fuel-window fill rate for a
        # large reduction in per-step socket traffic.
        self.sample_mod = max(1, int(sample_mod))
        # RSU objects are built in initialize_from_network once TraCI is live.
        self.rsus:               dict = {}
        self.edge_to_rsu:        dict = {}
        self.edge_speed_limits:  dict = {}
        self._edge_cost_calc          = None   # injected via set_edge_cost_calc()
        self.intersection = intersections
        # Track unique vehicles traversing each edge
        self.active_vehicles = {edge_id: {} for edge_id in edges}
        # Set of vehicle IDs currently subscribed (maintained to avoid duplicate
        # subscribe() calls and to clean up stale subscriptions each step).
        self._subscribed_vehicles: set = set()

    def set_edge_cost_calc(self, edge_cost_calc):
        """Wire in the EdgeCostCalculator so RSU can push fuel observations."""
        self._edge_cost_calc = edge_cost_calc

    def initialize_from_network(self, network_builder):
        graph         = network_builder.get_graph()
        intersections = self.intersection
        intersection_set = set(intersections)

        # Build a lookup: node_id -> RSU (created below) for fast dead-end fallback.
        # First pass: assign edges whose to_node is an intersection (primary assignment).
        incoming: dict = {node: [] for node in intersections}
        # We also need to know the from_node for each edge (for dead-end fallback).
        # On a MultiDiGraph graph.edges(data=True) yields (u, v, data) per parallel edge.
        edge_from_node: dict = {}   # edge_id -> from_node
        edge_to_node:   dict = {}   # edge_id -> to_node

        for u, v, data in graph.edges(data=True):
            eid = data["edge_id"]
            edge_from_node[eid] = u
            edge_to_node[eid]   = v
            # Some edges terminate at dead-end nodes that are not intersections;
            # they are handled in the second pass below.
            if v in incoming:
                incoming[v].append(eid)

        # Create RSUs for intersection nodes.
        for node in intersections:
            rsu = RSU(node, incoming[node], window_size=self.window_size)
            self.rsus[node] = rsu
            for eid in incoming[node]:
                self.edge_to_rsu[eid] = rsu

        # Second pass: assign dead-end-terminating edges to the RSU at their from_node.
        # This ensures every non-internal edge gets a dynamic weight instead of
        # falling back to static length forever.
        dead_end_assigned = 0
        for eid, to_node in edge_to_node.items():
            if eid in self.edge_to_rsu:
                continue   # already assigned in the primary pass
            from_node = edge_from_node.get(eid)
            if from_node in self.rsus:
                # Extend this RSU's connected_edges so it tracks the extra edge.
                rsu = self.rsus[from_node]
                if eid not in rsu.connected_edges:
                    rsu.connected_edges.append(eid)
                    # Initialise a rolling-window slot for the new edge.
                    rsu.edge_data[eid] = {
                        "vehicle_count":    deque(maxlen=rsu.window_size),
                        "avg_speed":        deque(maxlen=rsu.window_size),
                        "waiting_time":     deque(maxlen=rsu.window_size),
                        "stop_and_go_freq": deque(maxlen=rsu.window_size),
                        "fuel_consumption": deque(maxlen=rsu.window_size),
                        "co2_emissions":    deque(maxlen=rsu.window_size),
                        "queue_length":     deque(maxlen=rsu.window_size),
                        "occupancy":        deque(maxlen=rsu.window_size),
                    }
                self.edge_to_rsu[eid] = rsu
                dead_end_assigned += 1

        total_edges = len(edge_to_node)
        covered     = len(self.edge_to_rsu)
        print(f"[RSU] coverage: {covered} / {total_edges} non-internal edges have an RSU "
              f"({dead_end_assigned} assigned via from_node fallback for dead-end terminations).")
        for u, v, data in graph.edges(data=True):
            self.edge_speed_limits[data["edge_id"]] = data.get("speed_limit", 13.89)

        # Re-initialize active_vehicles to match the edges we actually track.
        self.active_vehicles = {edge_id: {} for edge_id in self.edge_to_rsu.keys()}

    def subscribe_edges(self):
        """
        Subscribe to bulk edge and initialise vehicle subscription tracking.
        MUST be called exactly once after traci.start() and before the first
        call to step(). After this call, step() issues only two TraCI bulk
        calls (getAllSubscriptionResults) instead of one per edge/vehicle.
        """
        for edge_id in self.edge_to_rsu:
            traci.edge.subscribe(edge_id, _EDGE_VARS)
        print(f"[RSU] subscribed {len(self.edge_to_rsu)} edges "
              f"(VEHICLE_ID_LIST + OCCUPANCY + HALTING_NUMBER + FUEL + MEAN_SPEED).")
        print(f"[RSU] vehicle sampling 1/{self.sample_mod}: "
              f"expect ~{self.sample_mod}x slower fuel-window fill")

    def clear_fuel_deques(self, edges):
        """Clear fuel/CO2 rolling windows for the given edges.

        Called at grade-mode activation so pre-grade EU4 measurements don't
        dilute the EU0 signal used by _decompose() to compute F.  After the
        clear, only post-activation EU0 vehicle trips are averaged.
        Also clears the per-traversal fuel window so fuel-objective routing
        likewise sees only post-activation costs.
        """
        for edge_id in edges:
            rsu = self.edge_to_rsu.get(edge_id)
            if rsu is not None and edge_id in rsu.edge_data:
                rsu.edge_data[edge_id]["fuel_consumption"].clear()
                rsu.edge_data[edge_id]["co2_emissions"].clear()
        if self._edge_cost_calc is not None:
            self._edge_cost_calc.clear_traversal_fuel(edges)

    def reset_for_new_state(self):
        """
        Re-arm the RSU layer after a ``traci.simulation.loadState()``.

        ``loadState`` atomically replaces the entire running simulation state --
        vehicles, positions, speeds, signals, and SUMO's RNG -- with a snapshot
        that was saved earlier.  This means:

        * Every vehicle ID that existed before the call may no longer exist, and
          new vehicle IDs may appear that were not present before.
        * The per-vehicle accumulators in ``active_vehicles`` (speeds, wait-times,
          fuel integrals) describe cars that no longer exist in the network, so
          they must be discarded completely.
        * The rolling-window observations in each ``RSU.edge_data`` deque were
          built from a traffic moment that no longer exists.  Averaging them into
          the weights for the new state would feed the router stale, wrong costs.
        * Edge subscriptions (``traci.edge.subscribe``) may be cleared by the
          loadState on the SUMO side.

        # VERIFY: whether traci.simulation.loadState clears SUMO-side edge and
        # vehicle subscriptions.  If getAllSubscriptionResults() returns stale
        # vehicle IDs from the previous state after a loadState, the step() guard
        # ``if not v_sub: continue`` already discards them safely (no subscription
        # data is available for IDs that SUMO has dropped), but explicitly
        # re-subscribing edges here is the safest approach regardless.

        What is intentionally NOT reset here
        -------------------------------------
        ``EdgeCostCalculator._fuel_baseline``: the free-flow fuel floor is a
        property of the edge geometry and typical traffic, not of one traffic
        snapshot.  Keeping it across trips within a seed models a persistently-
        deployed system that refines its knowledge over the campaign.  Call
        ``EdgeCostCalculator.reset()`` *between seeds*, not between trips.
        """
        # Discard all per-vehicle tracking (these vehicles no longer exist).
        self.active_vehicles = {edge_id: {} for edge_id in self.edge_to_rsu}
        # Drop the subscription-tracking set so step() re-subscribes any vehicle
        # it encounters from scratch (no duplicate-subscribe guard needed here).
        self._subscribed_vehicles = set()
        # Clear every RSU's rolling-window observations: they described a traffic
        # moment that loadState has now discarded.
        for rsu in self.rsus.values():
            rsu.clear()
        # Re-establish edge subscriptions so the very next step() call has fresh
        # bulk results from the newly-loaded network state.
        self.subscribe_edges()

    def step(self):
        """
        Collect per-edge traffic data using TraCI subscription bulk results.

        Two bulk calls replace the previous O(edges × vehicles) individual calls:
            edge_results = traci.edge.getAllSubscriptionResults()
            veh_results  = traci.vehicle.getAllSubscriptionResults()

        Vehicle subscriptions are created on first encounter and cleaned up
        when the vehicle is no longer on any tracked edge.
        """
        dt       = traci.simulation.getDeltaT()
        sim_time = traci.simulation.getTime()   # Change 2A: passed to record_traversal_fuel

        # --- ONE bulk call for all subscribed edge data ---
        edge_results = traci.edge.getAllSubscriptionResults()

        # --- ONE bulk call for all subscribed vehicle data ---
        veh_results  = traci.vehicle.getAllSubscriptionResults()

        for edge_id, rsu in self.edge_to_rsu.items():
            edge_data        = edge_results.get(edge_id, {})
            current_veh_ids  = set(edge_data.get(tc.LAST_STEP_VEHICLE_ID_LIST, []))
            previous_veh_ids = set(self.active_vehicles[edge_id].keys())

            # --- 1. TRACK ACTIVE VEHICLES ---
            for veh_id in current_veh_ids:
                if veh_id not in self.active_vehicles[edge_id]:
                    # Vehicle just entered the edge. Fix B: only sampled vehicles
                    # (deterministic 1/sample_mod hash) and egos get full per-vehicle
                    # tracking + a subscription. Unsampled vehicles get a lightweight
                    # sentinel so departed_veh_ids still detects them leaving the edge.
                    sampled = _is_ego(veh_id) or _is_sampled(veh_id, self.sample_mod)
                    if sampled:
                        self.active_vehicles[edge_id][veh_id] = {
                            "sampled":     True,
                            "speeds":      [],
                            "wait_time":   0,
                            "fuel":        0.0,   # accumulated mass (mg)
                            "co2":         0.0,   # accumulated mass (mg)
                            "halts":       0,     # rising-edge stop counter (Q3)
                            "was_stopped": False, # previous-step stopped flag
                        }
                        # Subscribe this vehicle if not already tracked.
                        # Results will be available from the NEXT step onwards;
                        # the first step on this edge is skipped (see NOTE in docstring).
                        if veh_id not in self._subscribed_vehicles:
                            traci.vehicle.subscribe(veh_id, _VEH_VARS)
                            self._subscribed_vehicles.add(veh_id)
                    else:
                        # Lightweight sentinel: track presence only.
                        self.active_vehicles[edge_id][veh_id] = {"sampled": False}

                # Accumulate only for sampled vehicles.
                v_data = self.active_vehicles[edge_id][veh_id]
                if not v_data.get("sampled"):
                    continue

                # Read from bulk subscription results (no individual TraCI call).
                v_sub = veh_results.get(veh_id)
                if not v_sub:
                    # No data yet (first step after subscribe); skip accumulation.
                    continue

                speed     = v_sub.get(tc.VAR_SPEED,           0.0)
                wait      = v_sub.get(tc.VAR_WAITING_TIME,     0.0)
                fuel_rate = v_sub.get(tc.VAR_FUELCONSUMPTION,  0.0)
                co2_rate  = v_sub.get(tc.VAR_CO2EMISSION,      0.0)

                # Accumulate this unique vehicle's stats.
                # rate (mg/s) * dt (s) = mass (mg) consumed this step.
                v_data["speeds"].append(speed)
                v_data["wait_time"] = max(v_data["wait_time"], wait)
                v_data["fuel"] += fuel_rate * dt
                v_data["co2"]  += co2_rate  * dt

                # Q3: count transitions INTO the stopped state (rising edge),
                # not a binary "ever stopped" flag. A vehicle stopping 3 times
                # yields halts == 3, giving gamma a meaningful count to weight.
                is_stopped = speed < 0.1
                if is_stopped and not v_data["was_stopped"]:
                    v_data["halts"] += 1
                v_data["was_stopped"] = is_stopped

            # --- 2. IDENTIFY DEPARTED VEHICLES ---
            departed_veh_ids = previous_veh_ids - current_veh_ids

            # --- 3. PUSH UNIQUE DATA TO RSU ---
            if departed_veh_ids:
                # Fix B: only SAMPLED departed vehicles carry per-vehicle stats and
                # feed the fuel cost model. Unsampled vehicles are invisible to the
                # fuel model (accepted sampling trade-off) but still cleaned up.
                sampled_departed = [
                    v for v in departed_veh_ids
                    if self.active_vehicles[edge_id].get(v, {}).get("sampled")
                ]

                agg_speed, agg_wait, agg_fuel, agg_co2, agg_halts = 0, 0, 0.0, 0.0, 0

                for veh_id in sampled_departed:
                    v_data = self.active_vehicles[edge_id][veh_id]

                    # True average speed of this vehicle over its entire transit
                    veh_avg_speed = (sum(v_data["speeds"]) / len(v_data["speeds"])
                                     if v_data["speeds"] else 0)

                    agg_speed  += veh_avg_speed
                    agg_wait   += v_data["wait_time"]
                    agg_fuel   += v_data["fuel"]
                    agg_co2    += v_data["co2"]
                    agg_halts  += v_data["halts"]

                    # Feed ONE fuel sample per completed trip: the vehicle's mean
                    # fuel RATE (mg/s) over its time on the edge. This weights every
                    # vehicle equally, instead of over-sampling slow/idling vehicles.
                    steps_on_edge = len(v_data["speeds"])
                    time_on_edge  = steps_on_edge * dt
                    if self._edge_cost_calc is not None and time_on_edge > 0:
                        mean_fuel_rate = v_data["fuel"] / time_on_edge  # mg/s
                        if mean_fuel_rate > 0:
                            self._edge_cost_calc.record_vehicle_fuel(edge_id, mean_fuel_rate)

                    # Also record the TOTAL traversal fuel for fuel-objective routing.
                    # One sample per traversal; never 0 (guard inside record_traversal_fuel).
                    # Change 2A: pass sim_time so max-age eviction can work correctly.
                    if self._edge_cost_calc is not None:
                        self._edge_cost_calc.record_traversal_fuel(edge_id, v_data["fuel"], sim_time)

                # Clean up memory for EVERY departed vehicle (sampled or not).
                for veh_id in departed_veh_ids:
                    del self.active_vehicles[edge_id][veh_id]

                # Instantaneous edge snapshot (queue_length/occupancy) always pushed.
                queue_length = edge_data.get(tc.LAST_STEP_VEHICLE_HALTING_NUMBER, 0)
                occupancy    = edge_data.get(tc.LAST_STEP_OCCUPANCY,              0.0)

                num_departed = len(sampled_departed)
                if num_departed > 0:
                    # Full trip-aggregate data point over sampled departed vehicles.
                    data_point = {
                        # --- trip aggregates over the sampled departed vehicles ---
                        "vehicle_count":    num_departed,
                        "avg_speed":        agg_speed / num_departed,
                        "waiting_time":     agg_wait  / num_departed,
                        # Q3: halts is now a mean stop-COUNT per vehicle (not a 0/1 flag).
                        # EdgeCostCalculator.compute_weight normalises by stop_ref before squaring.
                        "stop_and_go_freq": agg_halts / num_departed,
                        "fuel_consumption": agg_fuel  / num_departed,  # mean mass per trip (mg)
                        "co2_emissions":    agg_co2   / num_departed,  # mean mass per trip (mg)
                        # --- instantaneous edge snapshot from subscription ---
                        "queue_length": queue_length,
                        "occupancy":    occupancy,
                    }
                    rsu.update_edge_data(edge_id, data_point)
                else:
                    # Only unsampled vehicles departed: push the edge snapshot alone
                    # (occupancy/queue), preserving the road-state signal without
                    # fabricating per-vehicle aggregates. avg_speed=0 keeps GlobalMap
                    # from treating this as a fresh trip observation.
                    snapshot_point = {
                        "vehicle_count":    0,
                        "avg_speed":        0.0,
                        "waiting_time":     0,
                        "stop_and_go_freq": 0,
                        "fuel_consumption": 0,
                        "co2_emissions":    0,
                        "queue_length":     queue_length,
                        "occupancy":        occupancy,
                    }
                    rsu.update_edge_data(edge_id, snapshot_point)

            # --- 4. HANDLE EDGE CASES (Jams & Empty Roads) ---
            elif len(current_veh_ids) > 0:
                # JAM PREVENTION: vehicles present but none departing. If anyone is
                # stuck past the threshold, push a warning snapshot so the RSU's
                # rolling window reflects the congestion. Fix B: only sampled
                # vehicles carry wait_time; unsampled sentinels are skipped.
                _waits = [
                    v["wait_time"] for v in self.active_vehicles[edge_id].values()
                    if v.get("sampled")
                ]
                max_current_wait = max(_waits) if _waits else 0
                if max_current_wait > 30:
                    jam_data_point = {
                        "vehicle_count":    len(current_veh_ids),
                        "avg_speed":        0.1,
                        "waiting_time":     max_current_wait,
                        # Jam snapshot: 1.0 maps to stop_ref stops when normalised,
                        # i.e. a "saturated" stop-and-go signal.
                        "stop_and_go_freq": 1.0,
                        "fuel_consumption": 0,
                        "co2_emissions":    0,
                        "queue_length": edge_data.get(tc.LAST_STEP_VEHICLE_HALTING_NUMBER, 0),
                        "occupancy":    edge_data.get(tc.LAST_STEP_OCCUPANCY,              0.0),
                    }
                    rsu.update_edge_data(edge_id, jam_data_point)

            else:
                # EMPTY ROAD: push a clean data point so the RSU's rolling window
                # gradually clears out old traffic jams.
                empty_data_point = {
                    "vehicle_count":    0,
                    "avg_speed":        self.edge_speed_limits.get(edge_id, 13.89),
                    "waiting_time":     0,
                    "stop_and_go_freq": 0,
                    "fuel_consumption": 0,
                    "co2_emissions":    0,
                    "queue_length":     0,
                    "occupancy":        0,
                }
                rsu.update_edge_data(edge_id, empty_data_point)

        # --- 5. CLEAN UP STALE VEHICLE SUBSCRIPTIONS ---
        # Build the set of vehicles still active on any tracked edge this step.
        # Vehicles that are no longer active have either arrived at their
        # destination or left the tracked network; SUMO removes their
        # subscription data automatically, but we prune our tracking set to
        # keep _subscribed_vehicles from growing without bound.
        still_active: set = set()
        for veh_dict in self.active_vehicles.values():
            still_active.update(veh_dict.keys())
        self._subscribed_vehicles &= still_active

    def get_edge_stats(self, edge_id: str) -> dict:
        rsu = self.edge_to_rsu.get(edge_id)
        return rsu.get_average_stats(edge_id) if rsu else None