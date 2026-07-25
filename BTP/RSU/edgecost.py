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

    PHASE 3 — CHANGE 2A: CONFIGURABLE FUEL AGGREGATOR + MAX-AGE EVICTION
    The per-traversal fuel window (used by cost_mode="fuel") now stores
    (sim_time, mg) tuples instead of raw mg floats.  Three aggregators are
    supported via the `fuel_aggregator` parameter:
      "wmean"   — original recency-weighted mean (newest N× influence of oldest)
      "median"  — plain median; robust to single outlier traversals (DEFAULT)
      "trimmed" — mean after dropping the extreme min and max when n >= 4

    An optional max-age eviction drops samples older than `fuel_sample_max_age_s`
    (default 600 s sim-time) when a new sample is recorded, preventing stale
    observations from mixing with current traffic.  Eviction is write-time only.

    PHASE 3 — CHANGE 1: JUNCTION / TRANSITION FUEL PENALTY
    `get_junction_penalty(edge_id, m)` estimates the extra fuel cost of a
    stop-start event at the edge's upstream junction, weighted by the empirically
    observed probability of stopping.  Physics constants (documented below) are
    __init__ parameters so they can be overridden without subclassing.

    KNOWN DOUBLE-COUNTING (Change 1):
    `get_segment_fuel` already partially captures stop fuel when observed
    traversals themselves included a stop.  `get_junction_penalty` prices
    EXPECTED future stop-start on the ego's arrival; it does NOT subtract
    from `get_segment_fuel`.  This conservative over-count is a known v1
    limitation; it biases toward avoiding high-stop edges, which is the
    desired behaviour.  `--junction-weight 0` disables the penalty entirely.
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
                 debug_cfs=False,       # flag to capture decomposed C/F/S terms
                 fuel_window_size=10,   # per-traversal sample window for fuel-objective routing
                 nominal_fuel_rate_mg_s=50.0,  # cold-edge fallback rate (mg/s) at free-flow
                 # --- Change 2A: aggregator + max-age ---
                 fuel_aggregator="median",   # "wmean" | "median" | "trimmed"
                 fuel_sample_max_age_s=600.0, # drop samples older than this (sim-time seconds)
                 # --- Change 1: junction penalty physics ---
                 m_veh=1500.0,               # vehicle mass (kg)
                 eta_engine=0.30,            # engine thermal efficiency (dimensionless)
                 LHV_gasoline=43.5e6,        # lower heating value of gasoline (J/kg)
                 idle_overhead_factor=1.15,  # idle-time overhead at the stop (dimensionless)
                 junction_weight=1.0):       # multiplier on junction penalty; 0.0 disables
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
        # Edges whose baseline must not be updated (frozen at grade-mode activation
        # so the pre-degradation free-flow floor is preserved as the F denominator).
        self._frozen_baseline_edges: set = set()
        self._preseeded_edges: set = set()   # D1: edges filled by preseed (not observed)

        self.debug_cfs = debug_cfs
        self.cfs_records = []

        # --- Change 2A: Traversal-fuel window (for --cost-mode fuel) ---
        # Each entry is a (sim_time, total_fuel_mg) tuple for ONE vehicle completing
        # a full traversal.  Storing sim_time enables max-age eviction so stale
        # observations are not mixed with post-condition-change data.
        # Zeros are never appended (no zero-dilution).
        self._fuel_window_size = max(1, int(fuel_window_size))
        self._nominal_fuel_rate_mg_s = float(nominal_fuel_rate_mg_s)
        self._traversal_fuel: dict = {}   # edge_id -> deque[(sim_time, mg)]

        if fuel_aggregator not in ("wmean", "median", "trimmed"):
            raise ValueError(f"fuel_aggregator must be 'wmean', 'median', or 'trimmed'; got {fuel_aggregator!r}")
        self._fuel_aggregator      = fuel_aggregator
        self._fuel_sample_max_age_s = float(fuel_sample_max_age_s)

        # --- Change 1: Junction penalty physics constants ---
        self._m_veh               = float(m_veh)
        self._eta_engine          = float(eta_engine)
        self._LHV_gasoline        = float(LHV_gasoline)
        self._idle_overhead_factor = float(idle_overhead_factor)
        self.junction_weight      = float(junction_weight)

    def preseed_cold_baselines(self, sim_time=0.0):
        """Fix L + Fix Q: edge-blind cold baseline pre-seeding with structural regression.

        Called ONCE at grade-activation time by the harness (simulate.py), applied
        to EVERY edge in edge_lengths that does not yet have a locked baseline.
        This is intentionally edge-blind: it does NOT iterate degraded_edges and
        does NOT live inside RoadConditionManager's degradation logic, so both
        degraded and non-degraded cold edges receive the identical synthetic baseline
        before degradation is applied.

        Fix Q: structural preseed regression.
        Instead of assigning the same global median to all cold edges, the method
        fits a linear regression baseline_rate ≈ b0 + b1·speed_limit_mps on all
        currently OBSERVED (non-preseeded) edges, then uses the per-edge prediction
        for each cold edge.  Graph hygiene: edges with speed_limit < 5 m/s or
        length < 10 m are excluded from the regression training set (they are
        atypical short connectors whose rates would distort the fit).

        Falls back to the network-median (Fix L behaviour) if fewer than 10 observed
        baselines are available for regression.
        """
        import sys as _sys
        import numpy as _np

        # --- Fix Q: build regression training set from observed (non-preseeded) edges ---
        training = []
        for eid, rate in self._fuel_baseline.items():
            if eid in self._preseeded_edges:
                continue
            v_lim  = self.edge_speed_limits.get(eid, 0.0)
            length = self.edge_lengths.get(eid, 0.0)
            # Graph hygiene: skip atypical slow/short edges
            if v_lim < 5.0 or length < 10.0:
                continue
            if rate is not None and rate > 0:
                training.append((v_lim, rate))

        b0 = b1 = None
        if len(training) >= 10:
            try:
                X = _np.array([v for v, _ in training])
                Y = _np.array([r for _, r in training])
                A = _np.column_stack([_np.ones(len(X)), X])
                coeffs, _, _, _ = _np.linalg.lstsq(A, Y, rcond=None)
                b0, b1 = float(coeffs[0]), float(coeffs[1])
                _r2_mean = float(_np.mean(Y))
                _ss_res = float(_np.sum((Y - (b0 + b1 * X)) ** 2))
                _ss_tot = float(_np.sum((Y - _r2_mean) ** 2))
                _r2 = 1.0 - _ss_res / _ss_tot if _ss_tot > 0 else 0.0
                _sys.stderr.write(
                    f"[PRESEED_REGR] n_obs={len(training)} b0={b0:.2f} b1={b1:.4f} "
                    f"R²={_r2:.3f} at t={sim_time:.0f}\n")
            except Exception as _re:
                _sys.stderr.write(f"[PRESEED_REGR] regression failed ({_re}); using median\n")
                b0 = b1 = None

        nominal = self.cold_nominal_rate()
        n_preseeded = 0
        total = len(self.edge_lengths)
        for eid in self.edge_lengths:
            if eid not in self._fuel_baseline:
                if b0 is not None:
                    v_lim     = self.edge_speed_limits.get(eid, 13.89)
                    predicted = b0 + b1 * v_lim
                    value     = max(float(predicted), nominal)
                else:
                    value = nominal
                self._fuel_baseline[eid] = value
                self._preseeded_edges.add(eid)
                n_preseeded += 1

        _sys.stderr.write(
            f"[PRESEED] cold baselines pre-seeded for {n_preseeded}/{total} edges "
            f"at t={sim_time:.0f}"
            + (f" regression(b0={b0:.1f} b1={b1:.4f})" if b0 is not None
               else f" median={nominal:.1f}mg/s")
            + "\n")
        _sys.stderr.flush()

    def freeze_baseline(self, edge_id):
        """Prevent further EMA updates for edge_id.

        Called by RoadConditionManager at grade-mode activation so the
        pre-degradation free-flow fuel floor is preserved as the F denominator
        throughout the degradation window.

        Fix L: pre-seeding of cold edges is no longer done here (it was
        degradation-path-conditioned and thus edge-identity-aware).
        preseed_cold_baselines() is called edge-blindly by the harness before
        this method fires, so freeze_baseline now simply locks whatever baseline
        is already present (observed or pre-seeded).
        """
        import sys as _sys
        self._frozen_baseline_edges.add(edge_id)
        baseline_val = self._fuel_baseline.get(edge_id)
        seed_count   = len(self._fuel_seed.get(edge_id, []))
        _sys.stderr.write(f"[FREEZE_BASELINE] {edge_id}: baseline={baseline_val} seed_buf={seed_count} samples\n")
        _sys.stderr.flush()

    def unfreeze_baseline(self, edge_id):
        """Re-enable EMA updates after grade mode deactivates."""
        self._frozen_baseline_edges.discard(edge_id)

    def cold_nominal_rate(self, edge_id=None):
        """Best available fuel rate (mg/s) for a cold-edge fallback.

        Priority:
          1. This edge's own locked baseline (most accurate).
          2. Median of all locked baselines across the network — self-calibrates
             to the real fuel-rate distribution so unvisited edges aren't priced
             at the hardcoded 50 mg/s default, which is ~9x below the observed
             median of ~457 mg/s in the Monaco MoST scenario.
          3. _nominal_fuel_rate_mg_s (50 mg/s) when no baselines are locked yet
             (early warm-up).
        """
        if edge_id is not None:
            b = self._fuel_baseline.get(edge_id)
            if b:
                return b
        if self._fuel_baseline:
            vals = sorted(self._fuel_baseline.values())
            return vals[len(vals) // 2]
        return self._nominal_fuel_rate_mg_s

    # --- called once per completed vehicle trip by RSUManager (rate in mg/s) ---
    def record_vehicle_fuel(self, edge_id, fuel_rate):
        # Ignore non-physical / zero samples outright.
        if fuel_rate is None or fuel_rate <= 0:
            return

        # Frozen edges keep their pre-activation baseline so F stays valid.
        if edge_id in self._frozen_baseline_edges:
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

    # --- Fuel-objective routing: per-traversal fuel window (Change 2A) ---

    def record_traversal_fuel(self, edge_id, total_fuel_mg, sim_time=0.0):
        """Append the total fuel (mg) a single vehicle burned traversing edge_id.

        Called once per COMPLETED traversal; never called with 0 (callers must
        guard).  This is separate from record_vehicle_fuel (which feeds the F
        index EMA baseline) so the two cost models remain independent.

        Change 2A: stores (sim_time, mg) tuples instead of bare mg values.
        Before appending, evicts samples whose sim_time is older than
        fuel_sample_max_age_s from the front of the deque, preventing stale
        pre-condition-change observations from contaminating current estimates.
        Eviction is write-time only; reads use whatever is in the deque.
        """
        if total_fuel_mg is None or total_fuel_mg <= 0:
            return
        from collections import deque as _deque
        if edge_id not in self._traversal_fuel:
            self._traversal_fuel[edge_id] = _deque(maxlen=self._fuel_window_size)
        dq = self._traversal_fuel[edge_id]
        # Evict samples older than the max-age cutoff
        cutoff = float(sim_time) - self._fuel_sample_max_age_s
        while dq and dq[0][0] < cutoff:
            dq.popleft()
        dq.append((float(sim_time), float(total_fuel_mg)))

    def _cold_fuel_fallback(self, edge_id):
        """Free-flow fuel estimate for an edge with no traversal observations.

        Uses free_flow_time × fuel_rate, where fuel_rate comes from
        cold_nominal_rate() — the edge's own baseline if locked, otherwise the
        network-median baseline, otherwise the hardcoded nominal.
        """
        L     = self.edge_lengths.get(edge_id, 100.0)
        v_lim = self.edge_speed_limits.get(edge_id, 13.89)
        t_ff  = L / v_lim
        return self.cold_nominal_rate(edge_id) * t_ff

    def get_segment_fuel(self, edge_id):
        """Return the expected fuel (mg) for ONE vehicle to traverse edge_id.

        Change 2A: uses the configurable `fuel_aggregator` over the per-traversal
        window.  The window stores (sim_time, mg) tuples; mg values are extracted
        before aggregation.

        Aggregators:
          "wmean"   (original) — linear-recency-weighted mean; newest sample gets
                    weight N, oldest gets weight 1.  Sensitive to outlier traversals.
          "median"  (default)  — plain median; robust to single anomalous traversals.
          "trimmed"            — mean after dropping the single min and max when
                    n >= 4; plain mean for n < 4.

        Fallback: `_cold_fuel_fallback(edge_id)` when the window is empty.
                  Never returns 0 or infinity.
        """
        dq = self._traversal_fuel.get(edge_id)
        if not dq:
            return self._cold_fuel_fallback(edge_id)
        samples = [mg for _, mg in dq]   # extract mg; ignore sim_time here
        n = len(samples)
        if n == 0:
            return self._cold_fuel_fallback(edge_id)

        agg = self._fuel_aggregator
        if agg == "wmean":
            # Linear-recency-weighted mean: newest sample gets weight N,
            # oldest gets weight 1 (N = number of samples in the window).
            # This gives the most recent traversal ~18× the influence of the
            # oldest when the window is full at N=10.
            w_sum    = n * (n + 1) / 2    # sum of 1+2+…+n
            weighted = sum(s * (i + 1) for i, s in enumerate(samples))
            return weighted / w_sum
        elif agg == "median":
            s = sorted(samples)
            m = n // 2
            return s[m] if n % 2 else (s[m - 1] + s[m]) / 2.0
        elif agg == "trimmed":
            if n >= 4:
                trimmed = sorted(samples)[1:-1]   # drop single min and max
                return sum(trimmed) / len(trimmed)
            else:
                return sum(samples) / n
        else:
            # Unreachable (validated in __init__), but safe fallback
            return sum(samples) / n

    def clear_traversal_fuel(self, edge_ids):
        """Clear per-traversal fuel windows for the given edges.

        Called at grade-mode activation so pre-grade EU4 traversal costs do not
        dilute the post-activation EU0 signal used by fuel-objective routing.
        Works regardless of whether the deque stores bare mg or (sim_time, mg) tuples.
        """
        for eid in edge_ids:
            dq = self._traversal_fuel.get(eid)
            if dq is not None:
                dq.clear()

    # --- Change 1: Junction / transition fuel penalty ---

    def get_junction_penalty(self, edge_id, m):
        """Expected extra fuel (mg) of a stop-start event at the edge's upstream junction.

        Computes: penalty = p_stop × stop_start_fuel_mg

        p_stop (stop probability):
          Derived from RSU's stop_and_go_freq (mean stop count per vehicle).
          p_stop = clamp(stop_and_go_freq / 1.0, 0, 1).
          One or more mean stops → p=1 (conservative); zero stops → p=0.
          Returns 0 immediately when stop_and_go_freq <= 0 (cold / free-flowing edge).

        stop_start_fuel_mg (physics-based re-acceleration fuel):
          v_cruise = min(speed_limit, observed avg_speed if > 1 m/s else speed_limit)
          KE       = 0.5 × m_veh × v_cruise²          (Joules)
          fuel_kg  = KE / (eta_engine × LHV_gasoline)
          fuel_mg  = fuel_kg × 1e6 × idle_overhead_factor

        Constants (all __init__ parameters; defaults listed here):
          m_veh               = 1500.0 kg   — typical passenger car
          eta_engine          = 0.30         — engine thermal efficiency
          LHV_gasoline        = 43.5e6 J/kg  — lower heating value of gasoline
          idle_overhead_factor = 1.15        — accounts for idling time at stop

        Sanity check: at v = 13.89 m/s ≈ 50 km/h the formula yields ≈ 12 700 mg
        per full stop-start, consistent with the real-world 5–15 mL range.

        Known double-counting: get_segment_fuel already partially includes stop fuel
        from past traversal observations.  This penalty prices EXPECTED future
        stop behaviour on the ego's arrival, so the overlap is a conservative bias
        toward avoiding high-stop edges, not a systematic error.  Disable with
        junction_weight=0 (--junction-weight 0) to restore pre-Change-1 behaviour.
        """
        stop_freq = m.get("stop_and_go_freq", 0.0)
        if stop_freq <= 0.0:
            return 0.0   # cold edge or free-flowing — no penalty

        # p_stop: clamp stop_and_go_freq / 1.0 to [0, 1]
        p_stop = min(float(stop_freq) / 1.0, 1.0)

        # v_cruise: observed avg_speed if > 1 m/s, else speed_limit; capped by limit
        v_lim    = self.edge_speed_limits.get(edge_id, 13.89)
        v_obs    = m.get("avg_speed", 0.0)
        v_cruise = min(v_lim, v_obs if v_obs > 1.0 else v_lim)
        v_cruise = max(v_cruise, 1.0)   # floor at 1 m/s to keep computation positive

        # KE = ½ m_veh v²  (Joules)
        KE = 0.5 * self._m_veh * v_cruise ** 2

        # fuel_mg = (KE / (eta × LHV)) × 1e6 × overhead
        fuel_kg = KE / (self._eta_engine * self._LHV_gasoline)
        fuel_mg = fuel_kg * 1e6 * self._idle_overhead_factor

        return p_stop * fuel_mg

    @staticmethod
    def _clamp(x, lo=0.0, hi=1.0):
        return max(lo, min(hi, x))

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
            from traci_compat import traci
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

    def get_state(self):
        """Serialize learned ECC state for cross-arm sharing (Fix P).

        Returns a plain dict of picklable objects representing all learned
        state built during the phase-0 warmup.  The returned dict is passed
        to load_state() in each arm worker immediately after Simulation() is
        created, so the arm inherits the warmup's EMA baselines and traversal
        windows instead of cold-starting.

        _frozen_baseline_edges and _preseeded_edges are intentionally excluded:
        they are set by grade activation (preseed/freeze at t=21000), which
        happens AFTER warmup_end_time=20400, so they are always empty at the
        point where get_state() is called.
        """
        from collections import deque as _deque
        return {
            "fuel_baseline": dict(self._fuel_baseline),
            "fuel_seed": {k: list(v) for k, v in self._fuel_seed.items()},
            "traversal_fuel": {
                k: list(v) for k, v in self._traversal_fuel.items()
            },
        }

    def load_state(self, state, sim_time=0.0):
        """Restore ECC learned state serialized by get_state() (Fix P).

        Called in each arm worker after Simulation() creation but before
        subscribe_edges() / campaign start, so the arm starts with the same
        EMA baselines the warmup built rather than a cold slate.

        Pre-existing baselines (from the arm's own initialization) are
        overwritten.  The traversal deques are reconstructed with the same
        maxlen as the current window size.
        """
        import hashlib as _hl, pickle as _pkl, sys as _sys
        from collections import deque as _deque
        self._fuel_baseline = dict(state.get("fuel_baseline", {}))
        self._fuel_seed = {k: list(v) for k, v in state.get("fuel_seed", {}).items()}
        self._traversal_fuel = {}
        for eid, items in state.get("traversal_fuel", {}).items():
            dq = _deque(maxlen=self._fuel_window_size)
            dq.extend((float(t), float(mg)) for t, mg in items)
            self._traversal_fuel[eid] = dq
        n_base = len(self._fuel_baseline)
        n_trav = len(self._traversal_fuel)
        _sys.stderr.write(
            f"[PYSTATE] ECC loaded: {n_base} baseline edges, {n_trav} traversal windows\n"
        )
        _sys.stderr.flush()

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
        self._frozen_baseline_edges = set()
        self._traversal_fuel = {}
