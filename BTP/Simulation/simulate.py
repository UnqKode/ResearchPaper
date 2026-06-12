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

DRIFT-FREE A/B CAMPAIGN MODE  (run_checkpointed_campaign)
----------------------------------------------------------
An upgraded campaign that eliminates the time-drift bias described in
compare_routing.py. For each trip k, it calls traci.simulation.loadState()
with a pre-built reference checkpoint (Phase A of the checkpoint scheme), so
both the "sumo" and "ours" arms start trip k from a byte-identical background
traffic state. See the method docstring and compare_routing.build_checkpoints()
for the full design.
"""

import csv
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
        w = self.weights.get(edge_id, float("inf"))
        if w == float("inf"):
            L = self.calc.edge_lengths.get(edge_id, 100.0)
            v_lim = self.calc.edge_speed_limits.get(edge_id, 13.89)
            return L / v_lim
        return w


class Simulation:
    def __init__(self,
                 net_file,
                 reroute_interval=30,
                 dev_threshold=0.20,
                 imp_threshold=0.15,
                 alpha=1.0, beta=0.8, gamma=1.5,
                 # --- ego-vehicle configuration ---
                 ego_vehicle_id="ego",
                 ego_origin=None,     # edge ID; if set (with ego_dest), ego is injected
                 ego_dest=None,       # edge ID
                 ego_type="DEFAULT_VEHTYPE",
                 ego_depart="now",
                 # --- A/B comparison configuration ---
                 ego_routing="ours",  # "ours" -> our Dijkstra ; "sumo" -> SUMO's routing
                 ego_od_list=None,    # list[(origin_edge, dest_edge)] for run_od_campaign
                 debug_cfs=False,     # Phase 1 debug flag
                 road_condition_manager=None):
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
            debug_cfs=debug_cfs
        )
        self.rsu_manager.set_edge_cost_calc(self.calc)

        # --- 5. Glue ---
        self.global_map = GlobalMap(self.rsu_manager, self.calc)
        self.route_cfs_records = []

        self.reroute_interval = reroute_interval

        # --- ego state ---
        self.ego_id     = ego_vehicle_id
        self.ego_origin = ego_origin
        self.ego_dest   = ego_dest
        self.ego_type   = ego_type
        self.ego_depart = ego_depart
        self._ego_route_id = "ego_route"
        self._ego_injected = False
        self.dev_threshold = dev_threshold
        self.imp_threshold = imp_threshold
        self.active_egos = {}

        self.ego_metrics = {"depart_time": None, "arrive_time": None, "fuel_mg": 0.0, "reroutes": 0}
        self.route_snapshot = {"route": [], "saved_weights": {}, "timestamp": 0.0}
        self.last_dijkstra_time = 0.0

        self._finalize_init(ego_routing, ego_od_list)

    def _finalize_init(self, ego_routing, ego_od_list):
        # --- A/B campaign state ---
        if ego_routing not in ("ours", "sumo"):
            raise ValueError("ego_routing must be 'ours' or 'sumo'")
        self.ego_routing = ego_routing
        self.ego_od_list = ego_od_list or []
        self.trip_results = []

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
                self._evaluate_and_reroute()

            step += 1
            if max_steps is not None and step >= max_steps:
                break

        self._report_ego()
        return step

    # =================================================================
    # A/B campaign: one ego through all OD pairs, in sequence
    # =================================================================
    def run_od_campaign(self, per_trip_timeout=3000, warmup_steps=300, verbose=True,
                        progress_log_path=None, log_every=1):
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

        Parameters
        ----------
        per_trip_timeout : int
            Maximum simulation steps per trip before declaring "did not arrive".
        warmup_steps : int
            Background-only steps before the first ego injection.
        verbose : bool
            Print per-trip progress to stdout.
        progress_log_path : str or None
            If given, write one CSV row per trip to this file for live monitoring.
        log_every : int
            Write to the progress CSV every ``log_every`` trips (1 = all trips).

        Returns a list of per-trip dicts:
            {trip, origin, dest, arrived, depart_time, arrive_time,
             duration_s, fuel_mg, reroutes, skipped}
        and also stores it on self.trip_results.
        """
        dt = traci.simulation.getDeltaT()
        results = []

        # Activate TraCI subscriptions (must happen after traci.start, before first step).
        self.rsu_manager.subscribe_edges()

        # --- Optional per-trip progress CSV ---
        _csv_file, _csv_writer = None, None
        if progress_log_path is not None:
            _csv_file = open(progress_log_path, "w", newline="")
            _fieldnames = ["trip", "routing", "origin", "dest",
                           "arrived", "duration_s", "fuel_mg", "reroutes"]
            _csv_writer = csv.DictWriter(_csv_file, fieldnames=_fieldnames,
                                         extrasaction="ignore")
            _csv_writer.writeheader()

        def _log_rec(rec):
            if _csv_writer is not None and (rec["trip"] % log_every == 0):
                _csv_writer.writerow({
                    "trip":      rec["trip"],
                    "routing":   self.ego_routing,
                    "origin":    rec.get("origin", ""),
                    "dest":      rec.get("dest", ""),
                    "arrived":   rec.get("arrived"),
                    "duration_s": rec.get("duration_s"),
                    "fuel_mg":   rec.get("fuel_mg"),
                    "reroutes":  rec.get("reroutes"),
                })
                _csv_file.flush()

        try:
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
                    _log_rec(results[-1])
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
                        self._evaluate_and_reroute()

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
                _log_rec(rec)
                if verbose:
                    n_total = len(self.ego_od_list)
                    arrived = [r for r in results if r.get("arrived")]
                    cum_fuel = sum(r["fuel_mg"] for r in arrived
                                   if r["fuel_mg"] is not None)
                    cum_time = sum(r["duration_s"] for r in arrived
                                   if r["duration_s"] is not None)
                    if rec["arrived"]:
                        print(f"[{self.ego_routing}] run {k+1}/{n_total} | "
                              f"time={rec['duration_s']:.1f}s fuel={rec['fuel_mg']:.0f}mg "
                              f"reroutes={rec['reroutes']} | "
                              f"cumulative: {len(arrived)} arrived, "
                              f"{cum_time:.0f}s {cum_fuel:.0f}mg")
                    else:
                        print(f"[{self.ego_routing}] run {k+1}/{n_total} | did NOT arrive")

        finally:
            if _csv_file is not None:
                _csv_file.close()

        self.trip_results = results
        return results

    # =================================================================
    # Drift-free A/B campaign: per-trip checkpoint reload
    # =================================================================
    def run_checkpointed_campaign(self, checkpoints, per_trip_timeout=3000,
                                   rewarm_steps=240, seed=None,
                                   verbose=True, progress_log_path=None,
                                   log_every=1):
        """
        A/B campaign using pre-built reference-pass checkpoints for drift-free
        comparison of the two routing strategies.

        MOTIVATION -- why checkpoints eliminate drift
        ---------------------------------------------
        The legacy ``run_od_campaign`` injects one ego and runs all OD pairs
        back-to-back in a single continuous simulation.  The faster routing arm
        reaches trip *k* at an earlier simulation time than the slower arm, so
        by mid-campaign the two arms compare their egos against structurally
        different background traffic.  "Same seed" only guarantees the same
        initial random draw, not the same traffic state at trip *k*.  With
        N=100 pairs and ``--scale 3``, the cumulative time divergence between
        arms can reach thousands of simulation-seconds, making the savings
        figures unreliable.

        The checkpoint design fixes this: Phase A (``build_checkpoints`` in
        ``compare_routing.py``) runs one reference SUMO instance -- no ego,
        same seed -- and snapshots the ENTIRE simulation state (vehicles,
        positions, speeds, RNG) at N moments spaced ``checkpoint_spacing``
        seconds apart.  Phase B (this method) RELOADS the same checkpoint for
        trip *k* in BOTH the "sumo" and "ours" arms.  Because ``loadState``
        restores a byte-identical snapshot, the pre-ego traffic at trip *k* is
        provably identical between the two arms regardless of how quickly or
        slowly previous trips completed.

        STATE-PERSISTENCE POLICY
        ------------------------
        * RSU rolling-window observations and per-vehicle accumulators:
          RESET on every ``loadState`` via ``rsu_manager.reset_for_new_state()``.
          The loaded state has a completely different vehicle population, so
          stale observations from the previous trip would corrupt the dynamic
          weights used by the "ours" router.  The per-trip re-warm
          (``rewarm_steps``) repopulates the windows from scratch.

        * EdgeCostCalculator free-flow fuel baselines (``_fuel_baseline``):
          PERSIST across trips within a seed.  The free-flow fuel floor is a
          property of the edge geometry and typical traffic, not a property of
          one traffic moment.  Persisting it models a real deployed system that
          learns the baseline continuously.  It is reset BETWEEN seeds (via
          ``EdgeCostCalculator.reset()``) because different seeds may represent
          different time periods or demand patterns.

        TRIPINFO CAVEAT
        ---------------
        SUMO's ``--tripinfo-output`` assumes a single continuous run.  With
        repeated ``loadState`` calls, ego vehicles vanish on reload without
        recording a normal arrival, leaving the tripinfo file incomplete and
        garbled.  This method therefore measures each trip from the LIVE TraCI
        integration in ``_track_ego``:
          duration = arrive_time - depart_time
          fuel     = sum(getFuelConsumption * dt)  over all steps with ego present
        The caller (``run_scenario`` in compare_routing.py) must NOT parse
        tripinfo for checkpoint-mode runs.

        FAIRNESS FINGERPRINT
        --------------------
        Immediately before ego injection for trip *k*, this method records a
        lightweight state fingerprint:
          pre_inject_vcount  = traci.vehicle.getIDCount()
          pre_inject_possum  = round(sum(getLanePosition for all vehicles), 1)
        For a given (seed, k), these MUST be identical between the "sumo" and
        "ours" arms (both loaded the same checkpoint and ran the same re-warm
        with no ego present).  ``compare_pooled`` in compare_routing.py checks
        this and prints "fairness check: PASS" or lists mismatches.

        Do NOT call subscribe_edges() upfront here
        ------------------------------------------
        ``reset_for_new_state()`` calls ``subscribe_edges()`` internally after
        each ``loadState``, so the first call inside the loop serves as the
        upfront initialisation.  A premature ``subscribe_edges()`` before any
        ``loadState`` would be redundant (edges are subscribed against a state
        that is immediately overwritten) and could confuse SUMO's subscription
        tracking.

        Parameters
        ----------
        checkpoints : list[str]
            Ordered checkpoint file paths, one per OD pair.  Generated by
            ``build_checkpoints()`` in compare_routing.py (Phase A).
        per_trip_timeout : int
            Maximum simulation steps per trip before declaring "did not arrive".
        rewarm_steps : int
            Per-trip background-only steps after ``loadState`` before ego
            injection.  Should match or exceed the RSU window size (default 240).
        seed : int or None
            Traffic seed this campaign is running under; tagged into every
            per-trip record so the caller can pool results across seeds.
        verbose : bool
        progress_log_path : str or None
            If given, write one CSV row per trip (live progress).
        log_every : int
            Write to the progress CSV every ``log_every`` trips (1 = all trips).

        Returns
        -------
        list[dict]
            Per-trip dicts with the same schema as ``run_od_campaign`` plus:
            ``seed``, ``pre_inject_vcount``, ``pre_inject_possum``.
        """
        # The simulation step-length is a fixed property of the .sumocfg file;
        # it does not change across loadState calls, so reading it once here is safe.
        dt = traci.simulation.getDeltaT()

        results = []

        # ---- Optional per-trip progress CSV ----------------------------------------
        _csv_file, _csv_writer = None, None
        if progress_log_path is not None:
            _csv_file = open(progress_log_path, "w", newline="")
            _fieldnames = [
                "trip", "seed", "routing", "origin", "dest",
                "arrived", "duration_s", "fuel_mg", "reroutes",
                "pre_inject_vcount", "pre_inject_possum",
            ]
            _csv_writer = csv.DictWriter(_csv_file, fieldnames=_fieldnames,
                                         extrasaction="ignore")
            _csv_writer.writeheader()

        def _log_rec(rec):
            if _csv_writer is not None and (rec["trip"] % log_every == 0):
                _csv_writer.writerow({
                    "trip":              rec["trip"],
                    "seed":              seed,
                    "routing":           self.ego_routing,
                    "origin":            rec.get("origin", ""),
                    "dest":              rec.get("dest", ""),
                    "arrived":           rec.get("arrived"),
                    "duration_s":        rec.get("duration_s"),
                    "fuel_mg":           rec.get("fuel_mg"),
                    "reroutes":          rec.get("reroutes"),
                    "pre_inject_vcount": rec.get("pre_inject_vcount"),
                    "pre_inject_possum": rec.get("pre_inject_possum"),
                })
                _csv_file.flush()

        try:
            n_pairs = min(len(self.ego_od_list), len(checkpoints))
            if n_pairs < len(self.ego_od_list):
                print(f"[checkpointed] WARNING: only {len(checkpoints)} checkpoints "
                      f"for {len(self.ego_od_list)} OD pairs; truncating to {n_pairs}.")

            for k in range(n_pairs):
                origin, dest = self.ego_od_list[k]
                ckpt_path = checkpoints[k]

                # --- 1. Restore the reference traffic state ------------------------------
                # loadState() resets the ENTIRE simulation -- vehicles, positions, speeds,
                # queues, and SUMO's internal RNG -- to the saved snapshot.  Any ego from
                # the previous trip and all traffic accumulated during that trip are
                # discarded atomically.  Both arms receive the same snapshot for trip k,
                # so the only difference that can affect outcomes is who routes the ego.
                # VERIFY: traci.simulation.loadState is the correct TraCI API call.
                traci.simulation.loadState(ckpt_path)

                # --- 2. Re-arm the RSU layer ---------------------------------------------
                # The loaded state has a completely different vehicle population.  Clearing
                # the RSU rolling windows and re-subscribing edges ensures step() sees only
                # fresh data from the new state.  Fuel baselines are intentionally preserved
                # (see docstring STATE-PERSISTENCE POLICY above).
                self.rsu_manager.reset_for_new_state()

                # --- 3. Per-trip re-warm (background traffic only) -----------------------
                # Step ``rewarm_steps`` steps with no ego present.  This gives the RSU
                # rolling windows time to accumulate current data from the freshly-loaded
                # traffic so the "ours" router has accurate dynamic weights from its very
                # first reroute interval.  Both arms run the SAME re-warm steps on the
                # SAME checkpoint, so the traffic state at ego injection is provably
                # identical -- this is the core A/B fairness guarantee.
                early_drain = False
                for _ in range(rewarm_steps):
                    traci.simulationStep()
                    self.rsu_manager.step()
                    if traci.simulation.getMinExpectedNumber() <= 0:
                        if verbose:
                            print(f"[trip {k}] network drained during re-warm; "
                                  f"trip skipped.")
                        early_drain = True
                        break

                if early_drain:
                    rec = {
                        **self._trip_record(k, origin, dest, skipped=True),
                        "seed":              seed,
                        "pre_inject_vcount": None,
                        "pre_inject_possum": None,
                    }
                    results.append(rec)
                    _log_rec(rec)
                    continue

                # --- 4. Fairness fingerprint (captured before the ego enters) -----------
                # For a given (seed, k), this fingerprint MUST be identical between the
                # "sumo" and "ours" arms: both loaded the same checkpoint and ran the same
                # rewarm with no ego present, so the network state is byte-identical.
                # compare_pooled() in compare_routing.py checks this and flags any mismatch
                # as evidence that the checkpoint/loadState design failed.
                # VERIFY: traci.vehicle.getLanePosition is the correct per-vehicle call;
                # an alternative is to use subscription bulk results from rsu_manager.step()
                # on the last rewarm step, but direct calls are simpler for a fingerprint.
                pre_inject_vcount = traci.vehicle.getIDCount()
                _vid_list = traci.vehicle.getIDList()
                pre_inject_possum = round(
                    sum(traci.vehicle.getLanePosition(vid) for vid in _vid_list), 1
                )

                # --- 5. Reset ego metrics and inject the ego ----------------------------
                # Metrics are reset BEFORE injection so the very first step of the trip
                # loop writes into a clean dict (the ego may appear in getIDList() on the
                # same step it is added if SUMO processes it immediately).
                self.ego_metrics = {
                    "depart_time": None,
                    "arrive_time": None,
                    "fuel_mg":     0.0,
                    "reroutes":    0,
                }
                injected = self._inject_ego_trip(origin, dest, k)
                if not injected:
                    if verbose:
                        print(f"[trip {k}] SKIPPED (no route {origin} -> {dest})")
                    rec = {
                        **self._trip_record(k, origin, dest, skipped=True),
                        "seed":              seed,
                        "pre_inject_vcount": pre_inject_vcount,
                        "pre_inject_possum": pre_inject_possum,
                    }
                    results.append(rec)
                    _log_rec(rec)
                    continue

                # --- 6. Trip step loop --------------------------------------------------
                step = 0
                while True:
                    traci.simulationStep()
                    self.rsu_manager.step()
                    self._track_ego(dt)

                    # Reroute the ego with our Dijkstra every reroute_interval steps,
                    # but only when WE are routing it; "sumo" mode lets SUMO handle it.
                    if (self.ego_routing == "ours"
                            and step % self.reroute_interval == 0):
                        self.global_map.refresh(self.edges)
                        self.net_builder.update_graph_weights(self.global_map)
                        self._evaluate_and_reroute()

                    step += 1

                    if self.ego_metrics["arrive_time"] is not None:
                        break   # ego arrived normally
                    if step >= per_trip_timeout:
                        if verbose:
                            print(f"[trip {k}] TIMEOUT after {per_trip_timeout} steps "
                                  f"-- ego did not arrive.")
                        self._safe_remove_ego()
                        break
                    if traci.simulation.getMinExpectedNumber() <= 0:
                        if verbose:
                            print(f"[trip {k}] network drained before arrival.")
                        break

                # --- 7. Record the trip outcome ----------------------------------------
                rec = {
                    **self._trip_record(k, origin, dest, skipped=False),
                    "seed":              seed,
                    "pre_inject_vcount": pre_inject_vcount,
                    "pre_inject_possum": pre_inject_possum,
                }
                results.append(rec)
                _log_rec(rec)

                if verbose:
                    n_total = n_pairs
                    arrived_so_far = [r for r in results if r.get("arrived")]
                    cum_fuel = sum(r["fuel_mg"] for r in arrived_so_far
                                   if r["fuel_mg"] is not None)
                    cum_time = sum(r["duration_s"] for r in arrived_so_far
                                   if r["duration_s"] is not None)
                    if rec["arrived"]:
                        print(f"[{self.ego_routing}|seed={seed}] "
                              f"run {k+1}/{n_total} | "
                              f"time={rec['duration_s']:.1f}s "
                              f"fuel={rec['fuel_mg']:.0f}mg "
                              f"reroutes={rec['reroutes']} | "
                              f"cumulative: {len(arrived_so_far)} arrived, "
                              f"{cum_time:.0f}s {cum_fuel:.0f}mg")
                    else:
                        print(f"[{self.ego_routing}|seed={seed}] "
                              f"run {k+1}/{n_total} | did NOT arrive")

        finally:
            if _csv_file is not None:
                _csv_file.close()

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
            "trip":        k,
            "origin":      origin,
            "dest":        dest,
            "arrived":     arrived,
            "depart_time": None if skipped else m["depart_time"],
            "arrive_time": None if skipped else m["arrive_time"],
            "duration_s":  dur,
            "fuel_mg":     None if skipped else m["fuel_mg"],
            "reroutes":    0 if skipped else m["reroutes"],
            "skipped":     skipped,
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
            route_edges = []
            if self.ego_routing == "ours":
                cur_nodes = self.edge_to_nodes.get(origin)
                dest_nodes = self.edge_to_nodes.get(dest)
                if cur_nodes and dest_nodes:
                    self.global_map.refresh(self.edges)
                    self.net_builder.update_graph_weights(self.global_map)
                    onward = self.net_builder.get_dijkstra_route(cur_nodes[1], dest_nodes[1])
                    if onward:
                        route_edges = [origin] + onward

            if not route_edges:
                stage = traci.simulation.findRoute(origin, dest, vType=self.ego_type)
                if not stage.edges:
                    return False
                route_edges = list(stage.edges)

            traci.route.add(route_id, route_edges)
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
            
            # Capture the initial route snapshot for deviation tracking
            actual_route = traci.vehicle.getRoute(self.ego_id)
            self.route_snapshot = {
                "route": actual_route,
                "saved_weights": {e: self.global_map.get_weight(e) for e in actual_route},
                "timestamp": traci.simulation.getTime()
            }
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
            route_edges = []
            if self.ego_routing == "ours":
                cur_nodes = self.edge_to_nodes.get(self.ego_origin)
                dest_nodes = self.edge_to_nodes.get(self.ego_dest)
                if cur_nodes and dest_nodes:
                    self.global_map.refresh(self.edges)
                    self.net_builder.update_graph_weights(self.global_map)
                    onward = self.net_builder.get_dijkstra_route(cur_nodes[1], dest_nodes[1])
                    if onward:
                        route_edges = [self.ego_origin] + onward

            if not route_edges:
                stage = traci.simulation.findRoute(self.ego_origin, self.ego_dest,
                                                   vType=self.ego_type)
                if not stage.edges:
                    print(f"[ego] findRoute found no path "
                          f"{self.ego_origin} -> {self.ego_dest}; ego not injected.")
                    return
                route_edges = list(stage.edges)

            traci.route.add(self._ego_route_id, route_edges)
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
            
            # Capture initial route snapshot
            self.route_snapshot = {
                "route": route_edges,
                "saved_weights": {e: self.global_map.get_weight(e) for e in route_edges},
                "timestamp": traci.simulation.getTime()
            }
            print(f"[ego] injected '{self.ego_id}': "
                  f"{self.ego_origin} -> {self.ego_dest} ({len(route_edges)} edges).")
        except traci.TraCIException as e:
            print(f"[ego] injection failed: {e}")

    # -----------------------------------------------------------------
    def _track_ego(self, dt):
        if self.ego_id in traci.vehicle.getIDList():
            if self.ego_metrics["depart_time"] is None:
                self.ego_metrics["depart_time"] = traci.simulation.getTime()
            self.ego_metrics["fuel_mg"] += traci.vehicle.getFuelConsumption(self.ego_id) * dt
            
            cur_edge = traci.vehicle.getRoadID(self.ego_id)
            if cur_edge and not cur_edge.startswith(":"):
                if "driven_edges" not in self.ego_metrics:
                    self.ego_metrics["driven_edges"] = []
                if not self.ego_metrics["driven_edges"] or self.ego_metrics["driven_edges"][-1] != cur_edge:
                    self.ego_metrics["driven_edges"].append(cur_edge)

        if self.ego_id in traci.simulation.getArrivedIDList():
            self.ego_metrics["arrive_time"] = traci.simulation.getTime()

    # -----------------------------------------------------------------
    def _evaluate_and_reroute(self):
        """
        Deviation-based rerouting: only runs Dijkstra if the current remaining
        route has degraded by > dev_threshold, and only accepts the new route
        if it improves cost by > imp_threshold.
        """
        if self.ego_id not in traci.vehicle.getIDList():
            return  # not in the network yet, or already arrived
        try:
            cur_edge = traci.vehicle.getRoadID(self.ego_id)
            if not cur_edge or cur_edge.startswith(":"):
                return  # on an internal junction edge

            route = list(traci.vehicle.getRoute(self.ego_id))
            if not route:
                return
            dest_edge = route[-1]
            if cur_edge == dest_edge:
                return

            try:
                cur_idx = route.index(cur_edge)
                remaining_route = route[cur_idx:]
            except ValueError:
                return

            saved_remaining_cost = 0.0
            current_remaining_cost = 0.0
            
            for edge in remaining_route:
                cw = self.global_map.get_weight(edge)
                sw = self.route_snapshot["saved_weights"].get(edge, cw)
                saved_remaining_cost += sw
                current_remaining_cost += cw

            deviation_pct = 0.0
            if current_remaining_cost > saved_remaining_cost and saved_remaining_cost > 0:
                deviation_pct = (current_remaining_cost / saved_remaining_cost) - 1.0
            elif current_remaining_cost == float('inf') and saved_remaining_cost < float('inf'):
                deviation_pct = float('inf')

            print(f"[REROUTE_CHECK] vehicle={self.ego_id} currentEdge={cur_edge} "
                  f"remainingLen={len(remaining_route)} "
                  f"routeDeviation={deviation_pct*100:.1f}% "
                  f"currentRemainingCost={current_remaining_cost:.1f}")

            if deviation_pct <= self.dev_threshold:
                return  # Stable route, no Dijkstra needed

            cur_nodes = self.edge_to_nodes.get(cur_edge)
            dest_nodes = self.edge_to_nodes.get(dest_edge)
            if not cur_nodes or not dest_nodes:
                return

            src_node = cur_nodes[1]
            dst_node = dest_nodes[1]
            if src_node == dst_node:
                return

            now = traci.simulation.getTime()
            if now - self.last_dijkstra_time < 30.0:
                return
            self.last_dijkstra_time = now

            print(f"[DIJKSTRA_TRIGGER] vehicle={self.ego_id} reason=DEVIATION_THRESHOLD_EXCEEDED "
                  f"routeDeviation={deviation_pct*100:.1f}%")

            onward = self.net_builder.get_dijkstra_route(src_node, dst_node)
            if not onward:
                return

            candidate_route = [cur_edge] + onward
            candidate_cost = sum(self.global_map.get_weight(e) for e in candidate_route)

            improvement = 0.0
            if current_remaining_cost > 0:
                if current_remaining_cost == float('inf'):
                    improvement = 1.0 if candidate_cost < float('inf') else 0.0
                else:
                    improvement = (current_remaining_cost - candidate_cost) / current_remaining_cost

            print(f"[CANDIDATE_ROUTE] vehicle={self.ego_id} oldCost={current_remaining_cost:.1f} "
                  f"candidateCost={candidate_cost:.1f} improvement={improvement*100:.1f}%")

            accepted = improvement > self.imp_threshold
            if accepted:
                traci.vehicle.setRoute(self.ego_id, candidate_route)
                self.ego_metrics["reroutes"] += 1
                
                # Refresh snapshot
                actual_route = traci.vehicle.getRoute(self.ego_id)
                self.route_snapshot = {
                    "route": actual_route,
                    "saved_weights": {e: self.global_map.get_weight(e) for e in actual_route},
                    "timestamp": traci.simulation.getTime()
                }
                print(f"[DECISION] vehicle={self.ego_id} action=REROUTE")
            else:
                print(f"[DECISION] vehicle={self.ego_id} action=KEEP_CURRENT_ROUTE")

            # Phase 1b: Record decisive data on edges that matter
            if getattr(self.calc, "debug_cfs", False):
                step = int(now / traci.simulation.getDeltaT())
                route_edges = set(remaining_route) | set(candidate_route)
                for edge in route_edges:
                    m = self.rsu_manager.get_edge_stats(edge)
                    if m is None:
                        m = {}
                    d = self.calc._decompose(edge, m)
                    
                    self.route_cfs_records.append({
                        "ego": self.ego_id,
                        "decision_step": step,
                        "accepted": accepted,
                        "edge_id": edge,
                        "in_current": edge in remaining_route,
                        "in_candidate": edge in candidate_route,
                        "avg_speed": m.get("avg_speed", 0.0),
                        "occupancy": m.get("occupancy", 0.0),
                        "queue_length": m.get("queue_length", 0.0),
                        "waiting_time": m.get("waiting_time", 0.0),
                        "stop_and_go_freq": m.get("stop_and_go_freq", 0.0),
                        "fuel_consumption": m.get("fuel_consumption", 0.0),
                        "baseline_locked": d["baseline_locked"],
                        "baseline_value": d["baseline_value"],
                        "t_actual": d["t_actual"],
                        "C": d["C"],
                        "F": d["F"],
                        "S": d["S"],
                        "multiplier": d["multiplier"],
                        "weight": d["weight"]
                    })


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

    # =================================================================
    # Fixed-Departure Continuous Campaign (NEW PAIRED FUEL MODE)
    # =================================================================
    def run_fixed_departure_campaign(self, od_list, depart_start, depart_spacing,
                                     per_trip_timeout, ego_policy, reroute_interval,
                                     use_hysteresis, progress_log_path=None):
        """
        Injects a list of OD pairs into a single continuous simulation run at fixed
        scheduled times. Egos route according to `ego_policy`. Metrics are parsed
        offline from tripinfo; this method only records the departure drift footprint.
        """
        dt = traci.simulation.getDeltaT()
        results = []
        self.rsu_manager.subscribe_edges()

        # Temporarily override instance config for this mode
        original_routing = self.ego_routing
        original_interval = self.reroute_interval
        self.ego_routing = "sumo" if ego_policy == "sumo" else "ours"
        self.reroute_interval = reroute_interval

        # Optionally disable hysteresis for a matched cadence comparison
        orig_imp = self.imp_threshold
        orig_dev = self.dev_threshold
        if not use_hysteresis:
            self.imp_threshold = 0.0
            self.dev_threshold = 0.0

        # Build departure schedule: ego_id -> depart_time_s
        schedule = {}
        ego_states = {}
        for k, (origin, dest) in enumerate(od_list):
            depart_time_s = depart_start + k * depart_spacing
            vid = f"ego_{k}"
            schedule[vid] = depart_time_s
            ego_states[vid] = {
                "metrics": {"depart_time": None, "arrive_time": None, "fuel_mg": 0.0, "reroutes": 0, "driven_edges": []},
                "snapshot": {"route": [], "saved_weights": {}, "timestamp": 0.0},
                "last_dijkstra_time": 0.0,
                "origin": origin,
                "dest": dest,
                "depart_t": None,
            }

        active_egos = set()
        completed_egos = set()
        fast_forward_target = depart_start - 900.0 # 15 minutes rewarm

        while len(completed_egos) < len(od_list) and traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            sim_time = traci.simulation.getTime()

            if sim_time < fast_forward_target:
                # Fast forward without expensive Python processing
                continue

            self.rsu_manager.step()
            if hasattr(self, 'road_condition_manager') and self.road_condition_manager:
                self.road_condition_manager.step(sim_time)

            # Inject scheduled egos
            for k, (origin, dest) in enumerate(od_list):
                vid = f"ego_{k}"
                if vid in schedule and sim_time >= schedule[vid] and vid not in active_egos and vid not in completed_egos:
                    # Capture fingerprint
                    bg_vcount = traci.vehicle.getIDCount()
                    _vid_list = traci.vehicle.getIDList()
                    bg_possum = round(sum(traci.vehicle.getLanePosition(v) for v in _vid_list), 1)

                    # Temporarily bind pointers to THIS ego's entry
                    self.ego_id = vid
                    self.ego_metrics = ego_states[vid]["metrics"]
                    self.route_snapshot = ego_states[vid]["snapshot"]
                    self.last_dijkstra_time = ego_states[vid]["last_dijkstra_time"]
                    ego_states[vid]["depart_t"] = sim_time

                    injected = self._inject_ego_trip(origin, dest, k)

                    if injected:
                        active_egos.add(vid)
                        ego_states[vid]["metrics"] = self.ego_metrics
                        ego_states[vid]["snapshot"] = self.route_snapshot
                        ego_states[vid]["last_dijkstra_time"] = self.last_dijkstra_time
                        results.append({
                            "trip": k,
                            "seed": None, # Will be attached by caller
                            "origin": origin,
                            "dest": dest,
                            "depart_step": int(sim_time / dt),
                            "depart_t": sim_time,
                            "bg_vcount_at_depart": bg_vcount,
                            "bg_possum_at_depart": bg_possum,
                            "arrived": False,
                            "reroutes": 0,
                            "driven_edges": [],
                        })
                        print(f"[{ego_policy}] Injected {vid} at sim_time {sim_time:.1f}s (vcount={bg_vcount}, possum={bg_possum})")
                    else:
                        completed_egos.add(vid)
                        print(f"[{ego_policy}] SKIPPED {vid} (no route {origin} -> {dest})")

            # Track and Reroute active egos
            for vid in list(active_egos):
                # Bind pointers FIRST for the track loop
                self.ego_id = vid
                self.ego_metrics = ego_states[vid]["metrics"]
                self.route_snapshot = ego_states[vid]["snapshot"]
                self.last_dijkstra_time = ego_states[vid]["last_dijkstra_time"]

                self._track_ego(dt)

                # reroute_interval is in steps. We can use modulo on the current integer step count
                current_step = int(sim_time / dt)
                if self.ego_routing == "ours" and current_step % self.reroute_interval == 0:
                    # In _evaluate_and_reroute, it uses self.ego_id implicitly
                    self.global_map.refresh(self.edges)
                    self.net_builder.update_graph_weights(self.global_map)
                    self._evaluate_and_reroute()
                    
                # Update saved state
                ego_states[vid]["metrics"] = self.ego_metrics
                ego_states[vid]["snapshot"] = self.route_snapshot
                ego_states[vid]["last_dijkstra_time"] = self.last_dijkstra_time

                # Check completion
                if ego_states[vid]["metrics"]["arrive_time"] is not None:
                    active_egos.remove(vid)
                    completed_egos.add(vid)
                    # Update reroutes in results
                    for r in results:
                        if r["trip"] == int(vid.split("_")[1]):
                            r["arrived"] = True
                            r["reroutes"] = ego_states[vid]["metrics"]["reroutes"]
                            r["driven_edges"] = ego_states[vid]["metrics"].get("driven_edges", [])
                            break
                    print(f"[{ego_policy}] {vid} arrived.")
                    if progress_log_path:
                        try:
                            import csv
                            with open(progress_log_path, 'w', newline='') as f:
                                writer = csv.DictWriter(f, fieldnames=list(results[0].keys()))
                                writer.writeheader()
                                writer.writerows(results)
                            with open(progress_log_path.replace('progress_', 'cfs_debug_'), 'w', newline='') as f:
                                if self.calc.cfs_records:
                                    writer = csv.DictWriter(f, fieldnames=list(self.calc.cfs_records[0].keys()))
                                    writer.writeheader()
                                    writer.writerows(self.calc.cfs_records)
                            with open(progress_log_path.replace('progress_', 'route_cfs_'), 'w', newline='') as f:
                                if self.route_cfs_records:
                                    writer = csv.DictWriter(f, fieldnames=list(self.route_cfs_records[0].keys()))
                                    writer.writeheader()
                                    writer.writerows(self.route_cfs_records)
                        except Exception as e:
                            print(f"Failed to log progress: {e}")
                elif sim_time - schedule[vid] >= per_trip_timeout * dt:
                    print(f"[{ego_policy}] {vid} TIMEOUT after {per_trip_timeout} steps.")
                    self._safe_remove_ego()
                    active_egos.remove(vid)
                    completed_egos.add(vid)

        # Restore instance config
        self.ego_routing = original_routing
        self.reroute_interval = original_interval
        self.imp_threshold = orig_imp
        self.dev_threshold = orig_dev

        return results