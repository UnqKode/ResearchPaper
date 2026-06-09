"""
Simulation orchestrator: wires the four building blocks together and runs the
TraCI step loop.

    NetworkBuilder      -> the road graph + Dijkstra routing
    RSUManager          -> per-edge traffic stats from TraCI each step
    EdgeCostCalculator  -> stats  ->  scalar routing weight
    GlobalMap (here)    -> the glue: pulls each edge's smoothed stats from the
                           RSUs, runs them through the calculator, and exposes
                           get_weight(edge_id) so NetworkBuilder.update_graph_weights
                           can consume it unchanged.

EGO-VEHICLE MODE
----------------
Exactly ONE vehicle (the "ego") is steered at a time. It is routed either by
OUR dynamic-weight Dijkstra (ego_routing="ours") or by SUMO's own routing
(ego_routing="sumo"). Every other vehicle is left untouched. The RSUs still
observe ALL traffic, so the ego's decisions reflect real network congestion.

A/B CAMPAIGN MODE  (run_od_campaign)
------------------------------------
Given a list of (origin_edge, dest_edge) pairs, ONE ego vehicle runs through
every pair in sequence inside a single simulation -- arrive, then re-inject for
the next pair -- recording time and fuel per trip. Run the whole campaign once
with ego_routing="sumo" and once with ego_routing="ours" over the SAME od list
and the SAME traffic seed, and the only difference between the two runs is who
routes the ego. The driver script compare_routing.py does exactly that.
"""

import traci

from Routing.routingManager import NetworkBuilder
from RSU.rsuController import RSUManager
from RSU.edgecost import EdgeCostCalculator


class GlobalMap:
    """
    Adapter between the RSUs/calculator and the router.

    get_weight(edge_id) returns the most recently computed dynamic weight, or
    float('inf') for edges we have no usable data on yet -- which is exactly the
    signal NetworkBuilder.update_graph_weights uses to fall back to static length.
    """

    def __init__(self, rsu_manager, edge_cost_calc):
        self.rsu_manager = rsu_manager
        self.calc = edge_cost_calc
        self.weights = {}

    def refresh(self, edge_ids):
        """Recompute weights for every edge from its current smoothed stats."""
        for edge_id in edge_ids:
            stats = self.rsu_manager.get_edge_stats(edge_id)
            # Skip edges with no RSU (stats None/empty) or no traffic data yet
            # (avg_speed == 0 means the rolling window hasn't been populated).
            if not stats or stats.get("avg_speed", 0.0) <= 0.0:
                continue
            self.weights[edge_id] = self.calc.compute_weight(edge_id, stats)

    def get_weight(self, edge_id):
        return self.weights.get(edge_id, float("inf"))


class Simulation:
    def __init__(self,
                 net_file,
                 reroute_interval=30,
                 alpha=1.0, beta=0.8, gamma=1.5,
                 # --- ego-vehicle configuration ---
                 ego_vehicle_id="ego",
                 ego_origin=None,     # edge ID; if set (with ego_dest), ego is injected
                 ego_dest=None,       # edge ID
                 ego_type="DEFAULT_VEHTYPE",
                 ego_depart="now",
                 # --- A/B comparison configuration ---
                 ego_routing="ours",  # "ours" -> our Dijkstra ; "sumo" -> SUMO's routing
                 ego_od_list=None):   # list[(origin_edge, dest_edge)] for run_od_campaign
        # --- 1. Build the road graph (offline; uses sumolib, not TraCI) ---
        self.net_builder   = NetworkBuilder(net_file=net_file)
        self.graph         = self.net_builder.get_graph()
        self.intersections = self.net_builder.get_intersections()
        self.edges         = self.net_builder.get_all_edges()

        # --- 2. Static per-edge maps the calculator needs ---
        # self.graph is a MultiDiGraph, so edges(data=True) yields one (u,v,data)
        # entry per parallel edge. Duplicate edge_ids overwrite with the same
        # values (length and speed_limit are properties of the edge object itself),
        # so the resulting dicts are correct with no special handling needed.
        edge_lengths      = {}
        edge_speed_limits = {}
        for _u, _v, data in self.graph.edges(data=True):
            eid = data["edge_id"]
            edge_lengths[eid]      = data["length"]
            edge_speed_limits[eid] = data["speed_limit"]

        # edge_id -> (from_node, to_node), used when rerouting the ego
        self.edge_to_nodes = {}
        for edge in self.net_builder.net.getEdges():
            self.edge_to_nodes[edge.getID()] = (
                edge.getFromNode().getID(),
                edge.getToNode().getID(),
            )

        # --- 3. RSUs (one per intersection) ---
        self.rsu_manager = RSUManager(self.intersections, self.edges)
        self.rsu_manager.initialize_from_network(self.net_builder)

        # --- 4. Cost calculator, wired to the RSUs for fuel observations ---
        self.calc = EdgeCostCalculator(
            edge_lengths, edge_speed_limits,
            alpha=alpha, beta=beta, gamma=gamma,
        )
        self.rsu_manager.set_edge_cost_calc(self.calc)

        # --- 5. Glue ---
        self.global_map = GlobalMap(self.rsu_manager, self.calc)

        self.reroute_interval = reroute_interval

        # --- ego state ---
        self.ego_id     = ego_vehicle_id
        self.ego_origin = ego_origin
        self.ego_dest   = ego_dest
        self.ego_type   = ego_type
        self.ego_depart = ego_depart
        self._ego_route_id = "ego_route"
        self._ego_injected = False
        self.ego_metrics = {"depart_time": None, "arrive_time": None,
                            "fuel_mg": 0.0, "reroutes": 0}

        # --- A/B campaign state ---
        # ego_routing decides who steers the ego: our Dijkstra or SUMO.
        if ego_routing not in ("ours", "sumo"):
            raise ValueError("ego_routing must be 'ours' or 'sumo'")
        self.ego_routing = ego_routing
        self.ego_od_list = ego_od_list or []
        self.trip_results = []   # filled by run_od_campaign

    # =================================================================
    # Single-trip run (unchanged behaviour, now honours ego_routing)
    # =================================================================
    def run(self, max_steps=None):
        """Main loop for a single ego trip. Assumes traci.start(...) was called."""
        dt = traci.simulation.getDeltaT()
        # Activate TraCI subscriptions (must happen after traci.start, before first step).
        self.rsu_manager.subscribe_edges()
        self._maybe_inject_ego()

        step = 0
        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()

            # Collect this step's traffic data from ALL vehicles into the RSUs.
            self.rsu_manager.step()

            # Track the ego's outcome.
            self._track_ego(dt)

            # Periodically recompute weights and reroute ONLY the ego -- but only
            # when WE are the ones routing it. In "sumo" mode SUMO routes it.
            if self.ego_routing == "ours" and step % self.reroute_interval == 0:
                self.global_map.refresh(self.edges)
                self.net_builder.update_graph_weights(self.global_map)
                self._reroute_ego()

            step += 1
            if max_steps is not None and step >= max_steps:
                break

        self._report_ego()
        return step

    # =================================================================
    # A/B campaign: one ego through all OD pairs, in sequence
    # =================================================================
    def run_od_campaign(self, per_trip_timeout=3000, warmup_steps=300, verbose=True):
        """
        Run ONE ego vehicle through every (origin, dest) pair in self.ego_od_list,
        one trip after another, inside the single already-started simulation.

        For each pair: inject the ego, step until it arrives (or per_trip_timeout
        steps elapse), then record that trip's time and fuel. The ego is routed by
        our Dijkstra (ego_routing="ours") or by SUMO (ego_routing="sumo").

        Q2 WARM-UP: Before the first ego trip, the simulation is stepped for
        `warmup_steps` steps with only rsu_manager.step() active (no ego injected).
        This allows background traffic to populate _fuel_baseline in
        EdgeCostCalculator so the fuel index F is non-zero from the very first
        ego trip. Both the "ours" and "sumo" scenarios receive the SAME warm-up,
        preserving A/B fairness; only the ego's controller differs.

        Returns a list of per-trip dicts:
            {trip, origin, dest, arrived, depart_time, arrive_time,
             duration_s, fuel_mg, reroutes, skipped}
        and also stores it on self.trip_results.
        """
        dt = traci.simulation.getDeltaT()
        results = []

        # Activate TraCI subscriptions (must happen after traci.start, before first step).
        self.rsu_manager.subscribe_edges()

        # --- Q2: warm-up phase (no ego, background traffic only) ---
        if warmup_steps > 0 and verbose:
            print(f"[warm-up] running {warmup_steps} steps before first ego trip "
                  f"to seed fuel baselines ...")
        for _ in range(warmup_steps):
            traci.simulationStep()
            self.rsu_manager.step()
            if traci.simulation.getMinExpectedNumber() <= 0:
                if verbose:
                    print("[warm-up] network drained early; stopping warm-up.")
                break

        for k, (origin, dest) in enumerate(self.ego_od_list):
            injected = self._inject_ego_trip(origin, dest, k)
            if not injected:
                results.append(self._trip_record(k, origin, dest, skipped=True))
                if verbose:
                    print(f"[trip {k}] SKIPPED (no route {origin} -> {dest})")
                continue

            # Fresh per-trip metrics (reuses the same schema _track_ego writes to).
            self.ego_metrics = {"depart_time": None, "arrive_time": None,
                                "fuel_mg": 0.0, "reroutes": 0}

            step = 0
            while True:
                traci.simulationStep()
                self.rsu_manager.step()
                self._track_ego(dt)

                if self.ego_routing == "ours" and step % self.reroute_interval == 0:
                    self.global_map.refresh(self.edges)
                    self.net_builder.update_graph_weights(self.global_map)
                    self._reroute_ego()

                step += 1

                # Trip finished?
                if self.ego_metrics["arrive_time"] is not None:
                    break
                # Safety stops.
                if step >= per_trip_timeout:
                    if verbose:
                        print(f"[trip {k}] TIMEOUT after {per_trip_timeout} steps "
                              f"-- ego did not arrive.")
                    self._safe_remove_ego()
                    break
                if traci.simulation.getMinExpectedNumber() <= 0:
                    # Network fully drained (ego gone too) -- shouldn't normally
                    # happen mid-trip, but guard against an infinite loop.
                    if verbose:
                        print(f"[trip {k}] network drained before arrival.")
                    break

            rec = self._trip_record(k, origin, dest, skipped=False)
            results.append(rec)
            if verbose:
                n_total = len(self.ego_od_list)
                arrived = [r for r in results if r.get("arrived")]
                cum_fuel = sum(r["fuel_mg"] for r in arrived)
                cum_time = sum(r["duration_s"] for r in arrived)
                if rec["arrived"]:
                    print(f"[{self.ego_routing}] run {k+1}/{n_total} | "
                          f"time={rec['duration_s']:.1f}s fuel={rec['fuel_mg']:.0f}mg "
                          f"reroutes={rec['reroutes']} | "
                          f"cumulative: {len(arrived)} arrived, "
                          f"{cum_time:.0f}s {cum_fuel:.0f}mg")
                else:
                    print(f"[{self.ego_routing}] run {k+1}/{n_total} | did NOT arrive")

        self.trip_results = results
        return results

    # -----------------------------------------------------------------
    def _trip_record(self, k, origin, dest, skipped):
        m = self.ego_metrics
        arrived = (not skipped) and (m["arrive_time"] is not None)
        dur = None
        if arrived and m["depart_time"] is not None:
            dur = m["arrive_time"] - m["depart_time"]
        return {
            "trip": k,
            "origin": origin,
            "dest": dest,
            "arrived": arrived,
            "depart_time": None if skipped else m["depart_time"],
            "arrive_time": None if skipped else m["arrive_time"],
            "duration_s": dur,
            "fuel_mg": None if skipped else m["fuel_mg"],
            "reroutes": 0 if skipped else m["reroutes"],
            "skipped": skipped,
        }

    # -----------------------------------------------------------------
    def _inject_ego_trip(self, origin, dest, k):
        """
        Inject the ego for one trip with a UNIQUE id (ego_<k>) so SUMO never
        complains about reusing an id that already arrived. Initial route comes
        from SUMO's findRoute; in "ours" mode our Dijkstra takes over each
        interval, in "sumo" mode SUMO's rerouting device handles it.
        Returns True if injected, False if no route exists.
        """
        self.ego_id = f"ego_{k}"
        route_id = f"egoroute_{k}"
        try:
            stage = traci.simulation.findRoute(origin, dest, vType=self.ego_type)
            if not stage.edges:
                return False
            traci.route.add(route_id, stage.edges)
            traci.vehicle.add(self.ego_id, route_id,
                              typeID=self.ego_type, depart="now")
            # Opt the ego in/out of SUMO's rerouting device depending on mode.
            # (Background traffic should be launched with
            #  --device.rerouting.probability 1 so it behaves identically in both
            #  runs; only the ego's controller differs.)
            try:
                prob = "0" if self.ego_routing == "ours" else "1"
                traci.vehicle.setParameter(self.ego_id,
                                           "device.rerouting.probability", prob)
            except traci.TraCIException:
                pass
            return True
        except traci.TraCIException:
            return False

    # -----------------------------------------------------------------
    def _safe_remove_ego(self):
        """Remove a stuck ego (timed-out trip) so it doesn't linger."""
        try:
            if self.ego_id in traci.vehicle.getIDList():
                traci.vehicle.remove(self.ego_id)
        except traci.TraCIException:
            pass

    # -----------------------------------------------------------------
    def _maybe_inject_ego(self):
        """
        If ego_origin/ego_dest are given, add our controlled ego vehicle.
        Its initial route comes from SUMO's findRoute (just needs to be valid);
        our algorithm takes over via _reroute_ego when ego_routing == "ours". If
        no origin/dest is given we assume ego_vehicle_id already exists in the
        demand and just steer that one.
        """
        if self.ego_origin is None or self.ego_dest is None:
            print(f"[ego] no origin/dest set -- will steer existing vehicle "
                  f"'{self.ego_id}' if/when it appears in the demand.")
            return
        try:
            stage = traci.simulation.findRoute(self.ego_origin, self.ego_dest,
                                               vType=self.ego_type)
            if not stage.edges:
                print(f"[ego] findRoute found no path "
                      f"{self.ego_origin} -> {self.ego_dest}; ego not injected.")
                return
            traci.route.add(self._ego_route_id, stage.edges)
            traci.vehicle.add(self.ego_id, self._ego_route_id,
                              typeID=self.ego_type, depart=self.ego_depart)
            # In "ours" mode keep SUMO's rerouting device off the ego so the two
            # routing systems don't fight; in "sumo" mode let SUMO route it.
            try:
                prob = "0" if self.ego_routing == "ours" else "1"
                traci.vehicle.setParameter(self.ego_id,
                                           "device.rerouting.probability", prob)
            except traci.TraCIException:
                pass
            self._ego_injected = True
            print(f"[ego] injected '{self.ego_id}': "
                  f"{self.ego_origin} -> {self.ego_dest} ({len(stage.edges)} edges).")
        except traci.TraCIException as e:
            print(f"[ego] injection failed: {e}")

    # -----------------------------------------------------------------
    def _track_ego(self, dt):
        if self.ego_id in traci.vehicle.getIDList():
            if self.ego_metrics["depart_time"] is None:
                self.ego_metrics["depart_time"] = traci.simulation.getTime()
            self.ego_metrics["fuel_mg"] += traci.vehicle.getFuelConsumption(self.ego_id) * dt
        if self.ego_id in traci.simulation.getArrivedIDList():
            self.ego_metrics["arrive_time"] = traci.simulation.getTime()

    # -----------------------------------------------------------------
    def _reroute_ego(self):
        """
        Re-route ONLY the ego along the current cheapest path using our custom
        Dijkstra. Routed from the end of its current edge to the end of its
        destination edge, with the current edge prepended so SUMO accepts it.
        """
        if self.ego_id not in traci.vehicle.getIDList():
            return  # not in the network yet, or already arrived
        try:
            cur_edge = traci.vehicle.getRoadID(self.ego_id)
            if not cur_edge or cur_edge.startswith(":"):
                return  # on an internal junction edge

            route = traci.vehicle.getRoute(self.ego_id)
            if not route:
                return
            dest_edge = route[-1]
            if cur_edge == dest_edge:
                return

            cur_nodes  = self.edge_to_nodes.get(cur_edge)
            dest_nodes = self.edge_to_nodes.get(dest_edge)
            if cur_nodes is None or dest_nodes is None:
                return

            src_node = cur_nodes[1]   # end of the current edge
            dst_node = dest_nodes[1]  # end of the destination edge
            if src_node == dst_node:
                return

            onward = self.net_builder.get_dijkstra_route(src_node, dst_node)
            if not onward:
                return

            new_route = [cur_edge] + onward
            traci.vehicle.setRoute(self.ego_id, new_route)
            self.ego_metrics["reroutes"] += 1

        except traci.TraCIException:
            # Invalid/disconnected route this step -- keep the existing one.
            return

    # -----------------------------------------------------------------
    def _report_ego(self):
        m = self.ego_metrics
        if m["depart_time"] is None:
            print(f"[ego] vehicle '{self.ego_id}' never entered the network.")
            return
        dur = (m["arrive_time"] - m["depart_time"]) if m["arrive_time"] is not None else None
        print("\n--- EGO RESULT ---")
        print(f"  id:        {self.ego_id}")
        print(f"  routing:   {self.ego_routing}")
        print(f"  departed:  {m['depart_time']:.1f}s")
        print(f"  arrived:   {m['arrive_time'] if m['arrive_time'] is not None else 'did not arrive'}")
        if dur is not None:
            print(f"  duration:  {dur:.1f}s")
        print(f"  fuel:      {m['fuel_mg']:.1f} mg")
        print(f"  reroutes:  {m['reroutes']}")
