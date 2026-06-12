class EdgeCostCalculator:
    """
    Converts an edge's smoothed traffic metrics into a single scalar weight
    for Dijkstra / A* routing.

        weight = t_actual * (1 + alpha*C_congestion + beta*F_fuel + gamma*S_stopgo)

    - t_actual (seconds) = length / current avg speed, the physical backbone.
      It alone penalizes slow/congested edges and keeps the path total in
      interpretable time units (sum of edge weights ~= trip seconds).
    - The bracket is a dimensionless multiplier >= 1 inflating time by
      congestion, fuel, and stop-and-go penalties, each normalized to ~[0, 1].

    Properties:
    - Weights are always positive and finite  -> Dijkstra is valid.
    - weight >= t_free = length / speed_limit  -> an A* heuristic of
      (euclidean_distance / global_max_speed) never overestimates (admissible).
    - The multiplier is capped so a near-zero avg_speed in a jam cannot produce
      a runaway weight that destabilizes the search.

    vehicle_count is deliberately NOT used: occupancy already expresses density
    (count normalized by capacity), so a busy-but-flowing road is not penalized.

    STOP-AND-GO NORMALISATION (Q3):
    stop_and_go_freq from RSUManager is now a mean stop-COUNT per vehicle
    (not a 0/1 flag). It is divided by `stop_ref` (default 5 stops) before
    clamping to [0, 1] and squaring, so the squared term still bites in [0, 1]
    and the gamma coefficient retains the same scale as before.
    A vehicle stopping stop_ref or more times per edge traversal gets the full
    penalty (S = 1.0); one stop gives S = (1/stop_ref)^2 = 0.04 (with default).
    """

    def __init__(self,
                 edge_lengths,          # {edge_id: meters}
                 edge_speed_limits,     # {edge_id: m/s}
                 alpha=1.0,             # congestion weight
                 beta=0.8,              # fuel weight
                 gamma=1.5,             # stop-and-go weight (heaviest, fuel-driving)
                 w_ref=60.0,            # waiting-time reference (s) for normalization
                 veh_footprint=7.5,     # avg vehicle length + min gap (m) -> jam capacity
                 v_min=0.1,             # floor speed to avoid div-by-zero (m/s)
                 max_multiplier=10.0,   # cap on the penalty bracket
                 stop_ref=5.0,          # reference stop count for S normalisation (Q3)
                 baseline_seed_n=8,     # samples collected before locking the free-flow baseline
                 debug_cfs=False):      # flag to capture decomposed C/F/S terms
        self.edge_lengths      = edge_lengths
        self.edge_speed_limits = edge_speed_limits
        self.alpha, self.beta, self.gamma = alpha, beta, gamma
        self.w_ref          = w_ref
        self.veh_footprint  = veh_footprint
        self.v_min          = v_min
        self.max_multiplier = max_multiplier
        self.stop_ref       = stop_ref   # Q3: normalise raw stop count before squaring

        # Per-edge free-flow fuel-RATE baseline (mg/s), fed by record_vehicle_fuel.
        # Tracks downward fast / upward slow so it approximates the light-load floor.
        #
        # SEEDING FIX (was loophole #8): the baseline is NOT set by the first
        # vehicle to ever traverse an edge. Doing so let a single slow/idling
        # first car lock the floor artificially HIGH, after which the slow upward
        # EMA (a_up=0.05) could never bring it back down, so the fuel index F
        # stayed inflated forever and routes wrongly avoided previously-visited
        # edges. Instead we buffer the first `baseline_seed_n` samples per edge
        # and lock the baseline to their MINIMUM (the free-flow floor we actually
        # want), then switch to the asymmetric EMA. Until an edge has enough
        # samples its baseline is None -> compute_weight sets F = 0.0 (no fuel
        # penalty), which is the correct conservative behaviour.
        self._fuel_baseline = {}
        self._fuel_seed     = {}          # edge_id -> [first samples] until locked
        self._baseline_seed_n = max(1, int(baseline_seed_n))
        self._a_down = 0.20
        self._a_up   = 0.05
        
        self.debug_cfs = debug_cfs
        self.cfs_records = []

    # --- called once per completed vehicle trip by RSUManager (rate in mg/s) ---
    def record_vehicle_fuel(self, edge_id, fuel_rate):
        # Ignore non-physical / zero samples outright.
        if fuel_rate is None or fuel_rate <= 0:
            return

        cur = self._fuel_baseline.get(edge_id)
        if cur is None:
            # Not locked yet: accumulate seed samples instead of letting the very
            # first vehicle define the free-flow floor.
            buf = self._fuel_seed.setdefault(edge_id, [])
            buf.append(fuel_rate)
            if len(buf) >= self._baseline_seed_n:
                # Lock to the LOW end of the seed window (free-flow floor).
                self._fuel_baseline[edge_id] = min(buf)
                self._fuel_seed.pop(edge_id, None)
            return

        a = self._a_down if fuel_rate < cur else self._a_up
        self._fuel_baseline[edge_id] = (1.0 - a) * cur + a * fuel_rate

    @staticmethod
    def _clamp(x, lo=0.0, hi=1.0):
        return max(lo, min(hi, x))

    def compute_weight(self, edge_id, m):
        """
        m is the metrics dict for the edge:
        vehicle_count, avg_speed, waiting_time, stop_and_go_freq,
        fuel_consumption (mean mass per trip, mg), co2_emissions,
        queue_length, occupancy (percent 0..100).

        co2_emissions: collected by RSUManager for logging/analysis purposes
        only; it is intentionally NOT part of the routing weight here because
        CO2 is strongly correlated with fuel_consumption, which is already
        captured by the F (fuel index) term.
        """
    def _decompose(self, edge_id, m):
        L     = self.edge_lengths.get(edge_id, 100.0)
        v_lim = self.edge_speed_limits.get(edge_id, 13.89)

        # --- backbone: actual travel time (s) ---
        v        = max(m.get("avg_speed", 0.0), self.v_min)
        t_actual = L / v                      # always >= t_free = L / v_lim

        # --- congestion index: capacity-normalized, count excluded ---
        occ   = self._clamp(m.get("occupancy", 0.0) / 100.0)            # density 0..1
        q_jam = max(L / self.veh_footprint, 1.0)
        q     = self._clamp(m.get("queue_length", 0.0) / q_jam)         # queue fill 0..1
        w_time = self._clamp(m.get("waiting_time", 0.0) / self.w_ref)   # delay 0..1
        C = (occ + q + w_time) / 3.0

        # --- fuel index: current fuel RATE vs this edge's free-flow baseline ---
        baseline = self._fuel_baseline.get(edge_id)
        F = 0.0
        fuel_cons = m.get("fuel_consumption", 0.0)
        if baseline and baseline > 0 and fuel_cons > 0:
            cur_rate = fuel_cons / t_actual
            F = self._clamp(cur_rate / baseline - 1.0, 0.0, 2.0) / 2.0

        # --- stop-and-go: heaviest fuel driver, squared so it bites ---
        S = self._clamp(m.get("stop_and_go_freq", 0.0) / self.stop_ref) ** 2

        multiplier = 1.0 + self.alpha * C + self.beta * F + self.gamma * S
        multiplier = min(multiplier, self.max_multiplier)
        weight = t_actual * multiplier
        
        return {
            "t_actual": t_actual, "C": C, "F": F, "S": S,
            "multiplier": multiplier, "weight": weight,
            "baseline_locked": baseline is not None,
            "baseline_value": baseline if baseline else 0.0,
            "v": v
        }

    def compute_weight(self, edge_id, m):
        d = self._decompose(edge_id, m)
        weight = d["weight"]

        if self.debug_cfs:
            import traci
            try:
                sim_time = traci.simulation.getTime()
            except traci.TraCIException:
                sim_time = 0.0
                
            self.cfs_records.append({
                "edge_id": edge_id,
                "sim_time": sim_time,
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
                "alpha": self.alpha,
                "beta": self.beta,
                "gamma": self.gamma,
                "multiplier": d["multiplier"],
                "weight": d["weight"]
            })

        return weight

    def reset(self):
        """Forget all learned free-flow fuel baselines and seed buffers.

        STATE-PERSISTENCE POLICY: call this between SEEDS, never between trips
        within the same seed.

        The asymmetric EMA baseline (``_fuel_baseline``) and the seed-sample
        buffer (``_fuel_seed``) model a property of the edge -- its free-flow
        fuel consumption rate -- that is legitimately learned over many vehicle
        trips.  Across trips within one seed the router is behaving like a
        persistently-deployed system that continuously refines its knowledge, so
        keeping the baselines is both correct and beneficial (the fuel index F
        becomes more accurate as the campaign progresses).

        Across seeds, however, each seed may represent a different time period,
        demand pattern, or scenario variant.  Carrying a baseline learned under
        seed A into seed B could bias the fuel index in ways that are not
        attributable to the routing algorithm, so the baselines are reset at the
        seed boundary.
        """
        self._fuel_baseline = {}
        self._fuel_seed     = {}