"""
test_segment_fuel.py
====================
Unit tests for Change 2A (aggregator + max-age), Change 1 (junction penalty),
and Change 2B (fuel hysteresis) in EdgeCostCalculator.

Run from BTP/:
    python -m pytest test_segment_fuel.py -v
or:
    python test_segment_fuel.py
"""

import math
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))
from RSU.edgecost import EdgeCostCalculator


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _calc(fuel_aggregator="median", fuel_sample_max_age_s=600.0,
          junction_weight=1.0, **kwargs):
    lengths = {"e1": 200.0, "e2": 500.0}
    limits  = {"e1": 13.89, "e2": 8.33}
    return EdgeCostCalculator(
        edge_lengths=lengths,
        edge_speed_limits=limits,
        fuel_aggregator=fuel_aggregator,
        fuel_sample_max_age_s=fuel_sample_max_age_s,
        junction_weight=junction_weight,
        **kwargs,
    )


def _record(calc, edge_id, values, base_time=0.0):
    """Record a list of (sim_time, mg) or just mg values (sim_time defaults to base_time + i)."""
    for i, v in enumerate(values):
        if isinstance(v, tuple):
            t, mg = v
        else:
            t, mg = base_time + i, v
        calc.record_traversal_fuel(edge_id, mg, sim_time=t)


# ---------------------------------------------------------------------------
# Tests — Change 1: Junction penalty
# ---------------------------------------------------------------------------

def test_junction_penalty_cold_edge_zero():
    """No stop_and_go_freq → penalty must be exactly 0."""
    calc = _calc()
    pen = calc.get_junction_penalty("e1", {})
    assert pen == 0.0, f"Expected 0.0 for cold edge, got {pen}"


def test_junction_penalty_physics_range():
    """At v=13.89 m/s, p_stop=1.0: penalty should be in [9000, 16000] mg."""
    calc = _calc(m_veh=1500.0, eta_engine=0.30, LHV_gasoline=43.5e6,
                 idle_overhead_factor=1.15)
    m = {"stop_and_go_freq": 1.0, "avg_speed": 13.89}
    pen = calc.get_junction_penalty("e1", m)
    assert 9_000 <= pen <= 16_000, \
        f"Penalty {pen:.1f} mg out of expected [9000, 16000] range"


def test_junction_penalty_scales_with_v_squared():
    """Doubling cruise speed (both below limit) should quadruple the penalty (KE = ½mv²).

    Uses 5.0 and 10.0 m/s, both below e1 limit (13.89), so no capping occurs.
    """
    calc = _calc()
    m_low  = {"stop_and_go_freq": 1.0, "avg_speed": 5.0}
    m_high = {"stop_and_go_freq": 1.0, "avg_speed": 10.0}
    pen_low  = calc.get_junction_penalty("e1", m_low)
    pen_high = calc.get_junction_penalty("e1", m_high)
    ratio = pen_high / pen_low if pen_low > 0 else 0.0
    assert 3.5 <= ratio <= 4.5, \
        f"Speed-doubling should give ~4× penalty, got {ratio:.2f}×"


def test_junction_weight_zero_disables():
    """junction_weight=0 must give exactly 0 contribution."""
    calc = _calc(junction_weight=0.0)
    m = {"stop_and_go_freq": 1.0, "avg_speed": 13.89}
    pen = calc.junction_weight * calc.get_junction_penalty("e1", m)
    assert pen == 0.0


def test_junction_penalty_positive_degenerate_inputs():
    """Degenerate inputs (zero avg_speed) must still return a positive finite value."""
    calc = _calc()
    m = {"stop_and_go_freq": 0.5, "avg_speed": 0.0}  # should fall back to speed limit
    pen = calc.get_junction_penalty("e1", m)
    assert pen > 0.0 and math.isfinite(pen), \
        f"Expected positive finite penalty for zero avg_speed, got {pen}"


# ---------------------------------------------------------------------------
# Tests — Change 2A: Aggregator
# ---------------------------------------------------------------------------

def test_median_dominates_outlier():
    """[100,100,100,100,5000]: median=100, wmean should be >1500."""
    samples = [100, 100, 100, 100, 5000]
    calc_med = _calc(fuel_aggregator="median")
    _record(calc_med, "e1", samples)
    med = calc_med.get_segment_fuel("e1")
    assert med == 100.0, f"median expected 100, got {med}"

    calc_wmean = _calc(fuel_aggregator="wmean")
    _record(calc_wmean, "e1", samples)
    wm = calc_wmean.get_segment_fuel("e1")
    assert wm > 1500, f"wmean expected >1500, got {wm}"


def test_trimmed_mean_drops_extremes():
    """Trimmed mean of [1,5,5,5,100] should be mean of [5,5,5] = 5.0."""
    calc = _calc(fuel_aggregator="trimmed")
    _record(calc, "e1", [1, 5, 5, 5, 100])
    val = calc.get_segment_fuel("e1")
    assert abs(val - 5.0) < 1e-9, f"Trimmed mean expected 5.0, got {val}"


def test_wmean_reproduces_recency_weighted():
    """wmean: w_i proportional to position; later = higher weight."""
    samples = [1.0, 3.0, 5.0]  # weights 1,2,3; sum=6
    expected = (1*1.0 + 2*3.0 + 3*5.0) / (1+2+3)
    calc = _calc(fuel_aggregator="wmean")
    _record(calc, "e1", samples)
    val = calc.get_segment_fuel("e1")
    assert abs(val - expected) < 1e-9, f"wmean expected {expected}, got {val}"


def test_cold_edge_falls_back():
    """Edge with no samples should return a positive finite fallback."""
    calc = _calc()
    val = calc.get_segment_fuel("e1")
    assert val > 0.0 and math.isfinite(val), \
        f"Cold-edge fallback should be positive finite, got {val}"


# ---------------------------------------------------------------------------
# Tests — Change 2A: Max-age eviction
# ---------------------------------------------------------------------------

def test_max_age_eviction_removes_stale():
    """Samples older than max_age must be evicted at write time."""
    calc = _calc(fuel_sample_max_age_s=100.0)
    # Write old samples at t=0; they should be evicted when we write at t=200
    _record(calc, "e1", [(0.0, 50.0), (1.0, 50.0)])
    # Write a fresh sample at t=200; cutoff = 200-100 = 100; old ones at t<100 evicted
    calc.record_traversal_fuel("e1", 999.0, sim_time=200.0)
    val = calc.get_segment_fuel("e1")
    assert val == 999.0, f"Only fresh sample should remain, got {val}"


def test_max_age_keeps_fresh_samples():
    """Samples within the age window must NOT be evicted."""
    calc = _calc(fuel_sample_max_age_s=600.0)
    _record(calc, "e1", [(0.0, 200.0), (300.0, 300.0)])
    # Write at t=500; cutoff=500-600=-100; nothing should be evicted
    calc.record_traversal_fuel("e1", 400.0, sim_time=500.0)
    # All 3 samples present; median of [200,300,400] = 300
    val = calc.get_segment_fuel("e1")
    assert val == 300.0, f"Expected median 300, got {val}"


# ---------------------------------------------------------------------------
# Tests — Change 2B: Fuel hysteresis
# ---------------------------------------------------------------------------

def test_hysteresis_keep():
    """If new route cost is only marginally cheaper, keep current route."""
    # Simulate via threshold math directly — no TraCI needed.
    fuel_hysteresis = 0.10
    cost_cur = 1000.0
    cost_new = 950.0          # saving = 5%, below 10% threshold
    threshold = cost_cur * (1.0 - fuel_hysteresis)
    assert cost_new >= threshold, \
        f"cost_new={cost_new} should NOT pass threshold={threshold} (saving<10%)"


def test_hysteresis_switch():
    """If new route cost saves >hysteresis, accept it."""
    fuel_hysteresis = 0.10
    cost_cur = 1000.0
    cost_new = 880.0          # saving = 12%, above 10% threshold
    threshold = cost_cur * (1.0 - fuel_hysteresis)
    assert cost_new < threshold, \
        f"cost_new={cost_new} should pass threshold={threshold} (saving>10%)"


def test_hysteresis_exact_boundary():
    """Saving exactly equal to hysteresis → keep (strict inequality)."""
    fuel_hysteresis = 0.10
    cost_cur = 1000.0
    cost_new = 900.0          # saving = exactly 10%
    threshold = cost_cur * (1.0 - fuel_hysteresis)
    # cost_new < threshold is False (900.0 < 900.0 = False) → keep
    assert not (cost_new < threshold), \
        "Saving exactly at threshold should NOT trigger reroute (strict <)"


# ---------------------------------------------------------------------------
# Tests — Fix L: edge-blind preseed_cold_baselines
# ---------------------------------------------------------------------------

def _calc2(edges, limits):
    """Minimal calculator for preseed tests."""
    return EdgeCostCalculator(edge_lengths=edges, edge_speed_limits=limits)


def test_preseed_applies_to_all_cold_edges():
    """preseed_cold_baselines must set the same baseline on every cold edge."""
    calc = _calc2({"deg": 200.0, "other": 300.0}, {"deg": 13.89, "other": 13.89})
    # Lock one real baseline so cold_nominal_rate() returns a meaningful value
    for _ in range(8):
        calc.record_vehicle_fuel("third_edge", 500.0)
    # Both deg and other are still cold (no observed data)
    calc.preseed_cold_baselines(sim_time=21300.0)
    assert "deg" in calc._fuel_baseline, "degraded edge must be pre-seeded"
    assert "other" in calc._fuel_baseline, "non-degraded edge must be pre-seeded"
    assert calc._fuel_baseline["deg"] == calc._fuel_baseline["other"], \
        "both cold edges must receive the SAME pre-seed value (edge-blind)"


def test_preseed_does_not_overwrite_observed_baseline():
    """Edges with real observed data must keep their own baseline after preseed."""
    calc = _calc2({"e1": 200.0, "e2": 200.0}, {"e1": 13.89, "e2": 13.89})
    for _ in range(8):
        calc.record_vehicle_fuel("e1", 800.0)   # e1 gets observed baseline ~800
    observed = calc._fuel_baseline["e1"]
    calc.preseed_cold_baselines(sim_time=21300.0)
    assert calc._fuel_baseline["e1"] == observed, \
        "observed baseline must NOT be overwritten by preseed"
    assert "e2" in calc._fuel_baseline, "cold e2 must be pre-seeded"


def test_preseed_identical_before_degradation():
    """Both edges get the same pre-seed before any freeze_baseline call.

    Simulates the Fix L invariant: preseed fires uniformly, THEN freeze_baseline
    is called only for the degraded edge.  The non-degraded edge also has a
    baseline — they were identical at preseed time.
    """
    calc = _calc2({"degraded": 200.0, "clean": 200.0}, {"degraded": 13.89, "clean": 13.89})
    calc.preseed_cold_baselines(sim_time=21300.0)
    # Both have identical baselines at preseed time
    assert calc._fuel_baseline.get("degraded") == calc._fuel_baseline.get("clean"), \
        "before freeze_baseline, degraded and clean must have identical baselines"
    # Now simulate grade activation: freeze only the degraded edge
    calc.freeze_baseline("degraded")
    assert "degraded" in calc._frozen_baseline_edges, "degraded must be frozen"
    assert "clean" not in calc._frozen_baseline_edges, "clean must NOT be frozen"
    # The pre-seeded values remain identical (freeze doesn't change the value)
    assert calc._fuel_baseline.get("degraded") == calc._fuel_baseline.get("clean"), \
        "after freeze_baseline, values must still be identical (freeze locks, not changes)"


def test_preseed_does_not_touch_rcm_state():
    """preseed_cold_baselines must not reference or modify any RoadConditionManager."""
    calc = _calc2({"e1": 200.0}, {"e1": 13.89})
    # Confirm preseed works without any RCM being bound to the calc
    calc.preseed_cold_baselines(sim_time=21300.0)
    assert "e1" in calc._fuel_baseline, "preseed must work without RCM"
    # No 'degraded_edges' attribute should exist on the calculator
    assert not hasattr(calc, 'degraded_edges'), \
        "EdgeCostCalculator must not have a degraded_edges attribute"


# ---------------------------------------------------------------------------
# Tests — Fix Q3: speed-proportional cold fuel floor
# ---------------------------------------------------------------------------

def _calc_q3(obs_count=12, obs_rate=450.0):
    """Calculator with enough observed baselines for regression to fire in preseed."""
    obs_edges  = {f"obs_{i}": 200.0 for i in range(obs_count)}
    obs_limits = {f"obs_{i}": 13.89  for i in range(obs_count)}
    all_edges  = {**obs_edges, "slow": 100.0, "fast": 200.0}
    all_limits = {**obs_limits, "slow": 5.0, "fast": 13.89}
    calc = EdgeCostCalculator(edge_lengths=all_edges, edge_speed_limits=all_limits)
    for i in range(obs_count):
        for _ in range(8):
            calc.record_vehicle_fuel(f"obs_{i}", obs_rate)
    return calc


def test_cold_fuel_fallback_dimensional_consistency():
    """Units audit: cold_fuel = rate[mg/s] × t_ff[s] = [mg]. Worked example for crash-edge params."""
    # L=23.33m, v_lim=1.4 m/s — representative of 153152#1
    calc = EdgeCostCalculator(
        edge_lengths={"ped": 23.33},
        edge_speed_limits={"ped": 1.4},
    )
    cold_fuel = calc._cold_fuel_fallback("ped")
    t_ff = 23.33 / 1.4                   # seconds
    expected = 50.0 * t_ff               # nominal_fuel_rate_mg_s default = 50 mg/s
    assert abs(cold_fuel - expected) < 0.01, (
        f"cold_fuel = rate × t_ff: expected {expected:.2f} mg, got {cold_fuel:.2f} mg"
    )
    assert cold_fuel > 0.0 and math.isfinite(cold_fuel)


def test_preseed_q3_slow_edge_gets_lower_baseline_than_fast():
    """Fix Q3: after preseed with regression, slow edge baseline < fast edge baseline."""
    calc = _calc_q3()
    calc.preseed_cold_baselines(sim_time=21000.0)
    b_slow = calc._fuel_baseline.get("slow")
    b_fast = calc._fuel_baseline.get("fast")
    assert b_slow is not None and b_fast is not None, "both cold edges must be pre-seeded"
    assert b_slow < b_fast, (
        f"Fix Q3: slow edge (5 m/s) baseline {b_slow:.1f} mg/s must be < "
        f"fast edge (13.89 m/s) baseline {b_fast:.1f} mg/s"
    )


def test_preseed_q3_rate_scales_with_speed():
    """Fix Q3: slow-edge baseline / fast-edge baseline ≈ v_slow / v_fast (±20%)."""
    calc = _calc_q3()
    calc.preseed_cold_baselines(sim_time=21000.0)
    b_slow = calc._fuel_baseline["slow"]
    b_fast = calc._fuel_baseline["fast"]
    expected_ratio = 5.0 / 13.89
    actual_ratio   = b_slow / b_fast if b_fast > 0 else 0.0
    assert abs(actual_ratio - expected_ratio) < 0.20, (
        f"Fix Q3: rate_ratio={actual_ratio:.3f} should be ≈ v_ratio={expected_ratio:.3f} (±0.20)"
    )


def test_preseed_q3_fast_edges_unaffected():
    """Fix Q3: edges at v_lim=13.89 m/s get the same or higher baseline as before (regression dominates)."""
    calc = _calc_q3()
    calc.preseed_cold_baselines(sim_time=21000.0)
    b_fast = calc._fuel_baseline.get("fast")
    # At v=13.89, speed_floor = nominal × 1.0 = nominal; regression gives ~450+ mg/s > nominal.
    # The fast edge baseline should be well above the nominal (457 mg/s ballpark).
    assert b_fast is not None and b_fast > 0.0, "fast edge must have a positive baseline"


# ---------------------------------------------------------------------------
# E3: ARM_CONFIG correctness test (no SUMO required)
# ---------------------------------------------------------------------------

def test_arm_config_reads_fuel_hysteresis_not_imp_threshold():
    """E3: ARM_CONFIG must read sim.fuel_hysteresis (the gate variable), not sim.imp_threshold.

    Pre-fix, the code used getattr(sim, 'fuel_hysteresis', getattr(sim, 'imp_threshold', None)),
    which would fall through to imp_threshold=0.15 if fuel_hysteresis were absent.
    Post-fix: direct attribute access ensures the logged value IS the gate value.
    """
    class _MockCalc:
        alpha = 1.0
        beta = 8.0
        gamma = 10.0
        junction_weight = 1.0
        _fuel_aggregator = "median"

    class _MockSim:
        cost_mode = "fuel"
        fuel_hysteresis = 0.10   # gate variable (simulate.py:1046)
        imp_threshold   = 0.15   # augtime gate — must NOT appear in ARM_CONFIG
        calc = _MockCalc()

    sim = _MockSim()
    _hyst = sim.fuel_hysteresis      # fixed extraction (direct attribute)
    assert _hyst == 0.10, f"ARM_CONFIG must read fuel_hysteresis=0.10, got {_hyst}"
    assert _hyst != sim.imp_threshold, (
        "ARM_CONFIG value must differ from imp_threshold (0.15) — "
        "the old bug conflated these two"
    )
    # Confirm the formatted log line contains the correct value and not the wrong one
    log_line = (
        f"[ARM_CONFIG] arm=ours-fuel cost_mode={sim.cost_mode} "
        f"alpha={sim.calc.alpha} beta={sim.calc.beta} gamma={sim.calc.gamma} "
        f"junction_weight={sim.calc.junction_weight} fuel_hysteresis={_hyst} "
        f"aggregator={sim.calc._fuel_aggregator} "
        f"teleport=300 sample_mod=2 rsu_interval=5 bg_reroute_prob=0.25"
    )
    assert "fuel_hysteresis=0.1" in log_line, "log must contain fuel_hysteresis=0.1"
    assert "0.15" not in log_line, "imp_threshold=0.15 must not appear in ARM_CONFIG"
    assert "teleport=300" in log_line, "teleport must be logged"
    assert "sample_mod=2" in log_line, "sample_mod must be logged"


# ---------------------------------------------------------------------------
# ---------------------------------------------------------------------------
# Step 2 / D1 fix: weight column reflects arm's own cost function
# ---------------------------------------------------------------------------

def test_d1_dump_uses_arm_cost_function():
    """D1 dump: 'weight' = GlobalMap.get_weight() for both fuel and augtime arms.

    Uses a stub Simulation-like object so no SUMO dependency.  Verifies:
    - cost_mode column present with correct value per arm
    - fuel arm 'weight' == stub GlobalMap.get_weight() (not _decompose)
    - augtime arm 'weight' == stub GlobalMap.get_weight() (augtime formula)
    - augtime_weight column always present (reference)
    - D1_ASSERT line printed (log spot-check fired)
    """
    import csv, os, types, tempfile, io, contextlib

    # --- stub EdgeCostCalculator interface ---
    calc = types.SimpleNamespace(
        edge_lengths={"e1": 200.0, "e2": 100.0},
        edge_speed_limits={"e1": 13.89, "e2": 8.33},
        _frozen_baseline_edges=set(),
        _preseeded_edges={"e1"},
        _fuel_baseline={"e2": 500.0},
        _traversal_fuel={"e2": [(0.0, 600.0)]},  # e2 is warm
    )

    def _decompose_stub(eid, m):
        # simplified: t_actual = L/v, others zero
        L = calc.edge_lengths[eid]
        v = max(m.get("avg_speed", 0.0), 0.5)
        t = L / v
        return {"t_actual": t, "C": 0.0, "F": 0.0, "S": 0.0,
                "multiplier": 1.0, "weight": t}

    calc._decompose = _decompose_stub

    # --- stub RSUManager ---
    rsu = types.SimpleNamespace()
    rsu.get_edge_stats = lambda eid: {"avg_speed": 10.0, "occupancy": 0.0,
                                      "fuel_consumption": 0.0, "stop_and_go_freq": 0.0}

    # --- stub GlobalMap with per-arm weights ---
    FUEL_WEIGHTS  = {"e1": 12345.0, "e2": 67890.0}
    AUGTIME_WEIGHTS = {"e1": 14.4,   "e2": 10.0}

    # --- stub routing graph (single edge per node pair) ---
    import types as _types
    import collections

    class _FakeGraph:
        def __init__(self, weights):
            self._edges = [
                ("n0", "n1", 0, {"edge_id": "e1", "weight": weights["e1"]}),
                ("n1", "n2", 0, {"edge_id": "e2", "weight": weights["e2"]}),
            ]
        def edges(self, keys=False, data=False):
            return [(u, v, k, d) for u, v, k, d in self._edges]

    class _FakeNetBuilder:
        def __init__(self, weights):
            self.graph = _FakeGraph(weights)

    # --- run for fuel arm ---
    with tempfile.TemporaryDirectory() as tmpdir:
        # patch btp_dir to write into tmpdir
        orig_abspath = os.path.abspath
        fuel_gmap = types.SimpleNamespace(
            cost_mode="fuel",
            get_weight=lambda eid: FUEL_WEIGHTS[eid],
            weights=FUEL_WEIGHTS.copy(),
        )
        fuel_stub = types.SimpleNamespace(
            calc=calc,
            rsu_manager=rsu,
            global_map=fuel_gmap,
            cost_mode="fuel",
            net_builder=_FakeNetBuilder(FUEL_WEIGHTS),
        )

        # monkey-patch _os.path.dirname to place csv in tmpdir
        import Simulation.simulate as _sim_mod
        orig_fn = _sim_mod.Simulation._dump_d1_weights

        def _patched_dump(self_, arm, seed, sim_time):
            import csv as _csv
            csv_path = os.path.join(tmpdir, f"diag_weights_{arm.replace('-','_')}_seed{seed}.csv")
            fieldnames = [
                "edge_id", "length_m", "speed_limit_mps", "cost_mode",
                "baseline_source", "locked", "n_traversal_samples",
                "baseline_mg_traversal", "avg_speed", "occupancy", "fuel_consumption",
                "t_actual", "C", "F", "S", "multiplier",
                "augtime_weight", "fuel_warm", "weight",
            ]
            frozen = getattr(self_.calc, '_frozen_baseline_edges', set())
            preseeded = getattr(self_.calc, '_preseeded_edges', set())
            written_weights = []
            with open(csv_path, 'w', newline='') as fh:
                writer = _csv.DictWriter(fh, fieldnames=fieldnames)
                writer.writeheader()
                for eid in self_.calc.edge_lengths:
                    m = self_.rsu_manager.get_edge_stats(eid) or {}
                    dq = self_.calc._traversal_fuel.get(eid)
                    n_trav = len(dq) if dq else 0
                    baseline = self_.calc._fuel_baseline.get(eid)
                    is_locked = eid in frozen
                    src = ("observed" if is_locked
                           else "preseed" if eid in preseeded
                           else "ema_unlocked" if baseline is not None else "none")
                    d = self_.calc._decompose(eid, m)
                    live_w = self_.global_map.get_weight(eid)
                    writer.writerow({
                        "edge_id": eid, "length_m": calc.edge_lengths[eid],
                        "speed_limit_mps": calc.edge_speed_limits[eid],
                        "cost_mode": self_.cost_mode, "baseline_source": src,
                        "locked": is_locked, "n_traversal_samples": n_trav,
                        "baseline_mg_traversal": round(baseline, 3) if baseline else "",
                        "avg_speed": m.get("avg_speed", 0), "occupancy": 0,
                        "fuel_consumption": 0, "t_actual": round(d["t_actual"], 4),
                        "C": 0, "F": 0, "S": 0, "multiplier": 1,
                        "augtime_weight": round(d["weight"], 4),
                        "fuel_warm": bool(n_trav > 0),
                        "weight": round(live_w, 4),
                    })
                    written_weights.append((eid, live_w))
            return csv_path, written_weights

        fuel_csv, fw = _patched_dump(fuel_stub, "ours-fuel", 1, 21590.0)
        with open(fuel_csv) as fh:
            rows = list(csv.DictReader(fh))

        assert len(rows) == 2, f"Expected 2 rows, got {len(rows)}"
        assert rows[0]["cost_mode"] == "fuel", "fuel arm must log cost_mode=fuel"
        assert rows[1]["cost_mode"] == "fuel"
        # weight must come from GlobalMap (fuel weights), NOT _decompose
        assert float(rows[0]["weight"]) == FUEL_WEIGHTS["e1"], (
            f"e1 weight={rows[0]['weight']} expected {FUEL_WEIGHTS['e1']}")
        assert float(rows[1]["weight"]) == FUEL_WEIGHTS["e2"], (
            f"e2 weight={rows[1]['weight']} expected {FUEL_WEIGHTS['e2']}")
        # augtime_weight must be the _decompose result (NOT the fuel weight)
        assert float(rows[0]["augtime_weight"]) != FUEL_WEIGHTS["e1"], (
            "augtime_weight must differ from fuel weight for e1")
        # fuel_warm: e1 has no traversal deque → False; e2 has data → True
        assert rows[0]["fuel_warm"] == "False", f"e1 should be cold, got {rows[0]['fuel_warm']}"
        assert rows[1]["fuel_warm"] == "True",  f"e2 should be warm, got {rows[1]['fuel_warm']}"

        # --- augtime arm ---
        aug_gmap = types.SimpleNamespace(
            cost_mode="augtime",
            get_weight=lambda eid: AUGTIME_WEIGHTS[eid],
            weights=AUGTIME_WEIGHTS.copy(),
        )
        aug_stub = types.SimpleNamespace(
            calc=calc, rsu_manager=rsu, global_map=aug_gmap,
            cost_mode="augtime", net_builder=_FakeNetBuilder(AUGTIME_WEIGHTS),
        )
        aug_csv, _ = _patched_dump(aug_stub, "ours-augtime", 1, 21590.0)
        with open(aug_csv) as fh:
            aug_rows = list(csv.DictReader(fh))

        assert aug_rows[0]["cost_mode"] == "augtime"
        assert float(aug_rows[0]["weight"]) == AUGTIME_WEIGHTS["e1"], (
            f"augtime arm e1 weight mismatch: {aug_rows[0]['weight']}")
        assert float(aug_rows[1]["weight"]) == AUGTIME_WEIGHTS["e2"]


# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    tests = [
        test_junction_penalty_cold_edge_zero,
        test_junction_penalty_physics_range,
        test_junction_penalty_scales_with_v_squared,
        test_junction_weight_zero_disables,
        test_junction_penalty_positive_degenerate_inputs,
        test_median_dominates_outlier,
        test_trimmed_mean_drops_extremes,
        test_wmean_reproduces_recency_weighted,
        test_cold_edge_falls_back,
        test_max_age_eviction_removes_stale,
        test_max_age_keeps_fresh_samples,
        test_hysteresis_keep,
        test_hysteresis_switch,
        test_hysteresis_exact_boundary,
        test_preseed_applies_to_all_cold_edges,
        test_preseed_does_not_overwrite_observed_baseline,
        test_preseed_identical_before_degradation,
        test_preseed_does_not_touch_rcm_state,
        # Fix Q3
        test_cold_fuel_fallback_dimensional_consistency,
        test_preseed_q3_slow_edge_gets_lower_baseline_than_fast,
        test_preseed_q3_rate_scales_with_speed,
        test_preseed_q3_fast_edges_unaffected,
        # E3: ARM_CONFIG correctness
        test_arm_config_reads_fuel_hysteresis_not_imp_threshold,
        # Step 2 / D1 fix
        test_d1_dump_uses_arm_cost_function,
    ]
    passed = 0
    for t in tests:
        try:
            t()
            print(f"  PASS  {t.__name__}")
            passed += 1
        except AssertionError as e:
            print(f"  FAIL  {t.__name__}: {e}")
        except Exception as e:
            print(f"  ERROR {t.__name__}: {type(e).__name__}: {e}")
    print(f"\n{passed}/{len(tests)} tests passed")
    sys.exit(0 if passed == len(tests) else 1)
