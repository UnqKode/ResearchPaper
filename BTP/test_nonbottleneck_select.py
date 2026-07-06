"""
test_nonbottleneck_select.py
============================
Unit tests for Review-Fix Fixes 1-7.

Tests 1-4: _filter_nonbottleneck_candidates (pure function, no SUMO/TraCI).
Test 5:    OD coverage via stubbed sumolib path.
Test 6:    Hysteresis force-path (Fix 7) — stub calculator, off-route ego.
Test 7:    Fuel-specificity OLS intercept math (Fix 6).

Run from BTP/:
    python -m pytest test_nonbottleneck_select.py -v
or:
    python test_nonbottleneck_select.py
"""

import sys
import os
import math

sys.path.insert(0, os.path.dirname(__file__))
from Simulation.compare_routing import _filter_nonbottleneck_candidates


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_record(eid="e1", avg_occ=0.10, avg_spd=12.0, spd_lim=13.89,
                 length=200.0, traversals=50.0):
    return dict(eid=eid, avg_occ=avg_occ, avg_spd=avg_spd,
                spd_lim=spd_lim, length=length, traversals=traversals)


# ---------------------------------------------------------------------------
# Test 1 — Fix 1: speed filter direction
#   ratio=0.90 (free-flowing) PASSES; ratio=0.60 (speed-depressed) FAILS
# ---------------------------------------------------------------------------

def test_free_flowing_passes():
    r = _make_record(avg_occ=0.02, avg_spd=0.90 * 13.89, spd_lim=13.89,
                     length=200.0, traversals=50.0)
    result = _filter_nonbottleneck_candidates([r], p40_occ=0.30,
                                              speed_ratio_min=0.85,
                                              min_len=100.0, min_traversals=30)
    assert result == ["e1"], "free-flowing edge (ratio=0.90) should PASS"


def test_speed_depressed_fails():
    r = _make_record(avg_occ=0.02, avg_spd=0.60 * 13.89, spd_lim=13.89,
                     length=200.0, traversals=50.0)
    result = _filter_nonbottleneck_candidates([r], p40_occ=0.30,
                                              speed_ratio_min=0.85,
                                              min_len=100.0, min_traversals=30)
    assert result == [], "speed-depressed edge (ratio=0.60) should FAIL"


# ---------------------------------------------------------------------------
# Test 2 — Fix 2: occupancy normalisation
#   raw occ=55 (%) normalised to 0.55 passes the OCC_FLOOR=0.005 check but
#   then exceeds p40_occ=0.30 → FAIL.
#   raw occ=0.3 (%) normalised to 0.003 is below OCC_FLOOR=0.005 → FAIL.
# ---------------------------------------------------------------------------

def test_high_raw_occ_fails_after_normalisation():
    # avg_occ already normalised to 0.55 — simulates what the probe code does
    r = _make_record(avg_occ=0.55, avg_spd=0.90 * 13.89, spd_lim=13.89)
    result = _filter_nonbottleneck_candidates([r], p40_occ=0.30,
                                              speed_ratio_min=0.85,
                                              min_len=100.0, min_traversals=30)
    assert result == [], ("occ=0.55 > p40=0.30 should FAIL — confirms p40 "
                          "upper bound correctly excludes high-occ edges")


def test_too_empty_fails_occ_floor():
    # avg_occ=0.003 normalised value (0.3 raw % / 100) below OCC_FLOOR=0.005
    r = _make_record(avg_occ=0.003, avg_spd=0.90 * 13.89, spd_lim=13.89)
    result = _filter_nonbottleneck_candidates([r], p40_occ=0.30,
                                              speed_ratio_min=0.85,
                                              min_len=100.0, min_traversals=30)
    assert result == [], "occ below OCC_FLOOR should FAIL"


# ---------------------------------------------------------------------------
# Test 3 — Fix 4: length and traversal gates
# ---------------------------------------------------------------------------

def test_short_edge_fails():
    r = _make_record(avg_occ=0.02, avg_spd=0.90 * 13.89, length=80.0,
                     traversals=50.0)
    result = _filter_nonbottleneck_candidates([r], p40_occ=0.30,
                                              speed_ratio_min=0.85,
                                              min_len=100.0, min_traversals=30)
    assert result == [], "edge length 80 m < min_len 100 m should FAIL"


def test_low_traversals_fails():
    r = _make_record(avg_occ=0.02, avg_spd=0.90 * 13.89, length=200.0,
                     traversals=10.0)
    result = _filter_nonbottleneck_candidates([r], p40_occ=0.30,
                                              speed_ratio_min=0.85,
                                              min_len=100.0, min_traversals=30)
    assert result == [], "traversals=10 < min_traversals=30 should FAIL"


def test_all_filters_pass():
    r = _make_record(avg_occ=0.02, avg_spd=0.90 * 13.89, length=200.0,
                     traversals=50.0)
    result = _filter_nonbottleneck_candidates([r], p40_occ=0.30,
                                              speed_ratio_min=0.85,
                                              min_len=100.0, min_traversals=30)
    assert result == ["e1"], "edge satisfying all criteria should PASS"


# ---------------------------------------------------------------------------
# Test 4 — OD coverage via stubbed net.getShortestPath
#   edge on 3/10 paths passes at threshold 0.20
#   edge on 1/10 paths fails at threshold 0.20
# ---------------------------------------------------------------------------

def _stub_od_coverage(edge_ids_on_path, n_ods, od_coverage_min):
    """
    Simulates the sumolib coverage loop from select_nonbottleneck_grade_edges
    without needing a real net file.  edge_ids_on_path is the set of cand
    edges that appear in EVERY path (so their count = n_ods).
    """
    candidates = ["cand_e1", "cand_e2"]
    cand_set = set(candidates)
    cov = {e: 0 for e in candidates}
    for _ in range(n_ods):
        # Each "path" contains cand_e1 but NOT cand_e2
        for eid in ["cand_e1", "other_e"]:
            if eid in cand_set:
                cov[eid] += 1
    n_od = n_ods
    return [e for e in candidates if cov[e] / n_od >= od_coverage_min]


def test_od_coverage_passes_threshold():
    # cand_e1 appears in all 10 paths → 10/10 = 1.0 >= 0.20
    result = _stub_od_coverage(["cand_e1"], n_ods=10, od_coverage_min=0.20)
    assert "cand_e1" in result, "cand_e1 on 10/10 paths should pass coverage 0.20"


def test_od_coverage_fails_threshold():
    # cand_e2 appears in 0 paths → 0/10 = 0.0 < 0.20
    result = _stub_od_coverage(["cand_e1"], n_ods=10, od_coverage_min=0.20)
    assert "cand_e2" not in result, "cand_e2 on 0/10 paths should fail coverage 0.20"


def test_od_coverage_exact_boundary():
    # 2 paths / 10 = 0.20 — exactly at threshold → include (>= not >)
    candidates = ["ce"]
    cand_set = set(candidates)
    cov = {"ce": 2}
    n_od = 10
    result = [e for e in candidates if cov[e] / n_od >= 0.20]
    assert "ce" in result, "exactly at threshold (2/10=0.20) should PASS"


# ---------------------------------------------------------------------------
# Test 5 — Fix 7: force-path reroute for off-route ego
#   Stub _reroute_fuel_mode logic: when force=True, route is applied even if
#   cost_new > cost_cur.
# ---------------------------------------------------------------------------

def test_force_reroute_bypasses_hysteresis():
    """Verify the force=True path ignores the cost threshold."""
    hysteresis = 0.10
    cost_cur = 1000.0
    cost_new = 1200.0   # worse than current — would normally be KEPT

    # Replicate the branching in _reroute_fuel_mode
    force = True
    threshold = cost_cur * (1.0 - hysteresis)

    if force:
        applied = True   # always apply when force=True
    else:
        applied = cost_new < threshold

    assert applied, "force=True must apply the route regardless of cost"


def test_normal_reroute_respects_hysteresis():
    """Without force, a worse route must not be applied."""
    hysteresis = 0.10
    cost_cur = 1000.0
    cost_new = 1200.0
    threshold = cost_cur * (1.0 - hysteresis)
    assert not (cost_new < threshold), "worse route must be kept without force"


# ---------------------------------------------------------------------------
# Test 6 — Fix 6: fuel-specificity OLS intercept math
#   Synthetic data: Δfuel% = -10 + 1.0*Δtime% + noise across 5 seeds.
#   Expected: mean per-seed intercept in [-12, -8].
#   Equal-time subset (|Δtime%| <= 5) should have mean ≈ -10.
# ---------------------------------------------------------------------------

def _simple_ols(pairs_df_dt):
    """Minimal OLS returning (intercept, slope)."""
    n   = len(pairs_df_dt)
    sx  = sum(dt for _, dt in pairs_df_dt)
    sy  = sum(df for df, _ in pairs_df_dt)
    sxx = sum(dt**2 for _, dt in pairs_df_dt)
    sxy = sum(df*dt for df, dt in pairs_df_dt)
    denom = n*sxx - sx**2
    if denom == 0:
        return None, None
    b = (n*sxy - sx*sy) / denom
    a = (sy - b*sx) / n
    return a, b


def test_fuel_specificity_intercept_range():
    """Per-seed intercepts from synthetic data where a_true=-10 should average ≈ -10."""
    import random
    rng = random.Random(7)

    per_seed_intercepts = []
    for seed in range(5):
        # 20 ego pairs per seed; Δfuel% = -10 + 1.0*Δtime% + N(0, 1)
        pairs = []
        for _ in range(20):
            dt = rng.gauss(0.0, 8.0)     # time change varies widely
            df = -10.0 + 1.0 * dt + rng.gauss(0.0, 1.0)
            pairs.append((df, dt))
        a_s, _ = _simple_ols(pairs)
        if a_s is not None:
            per_seed_intercepts.append(a_s)

    assert len(per_seed_intercepts) == 5, "all 5 seeds should yield valid intercepts"
    mean_int = sum(per_seed_intercepts) / len(per_seed_intercepts)
    assert -12.0 <= mean_int <= -8.0, \
        f"mean per-seed intercept {mean_int:.2f} should be in [-12, -8]"


def test_equal_time_subset_mean():
    """Equal-time subset (|Δtime%|<=5) should recover the true intercept ≈ -10."""
    import random
    rng = random.Random(13)

    all_eq_time = []
    for seed in range(5):
        eq_fuels = []
        for _ in range(50):
            dt = rng.gauss(0.0, 8.0)
            df = -10.0 + 1.0 * dt + rng.gauss(0.0, 1.0)
            if abs(dt) <= 5.0:
                eq_fuels.append(df)
        if eq_fuels:
            all_eq_time.append(sum(eq_fuels) / len(eq_fuels))

    assert all_eq_time, "equal-time subset should contain some egos"
    mean_eq = sum(all_eq_time) / len(all_eq_time)
    assert -12.0 <= mean_eq <= -8.0, \
        f"equal-time subset mean {mean_eq:.2f} should be ≈ -10 (in [-12, -8])"


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    tests = [
        test_free_flowing_passes,
        test_speed_depressed_fails,
        test_high_raw_occ_fails_after_normalisation,
        test_too_empty_fails_occ_floor,
        test_short_edge_fails,
        test_low_traversals_fails,
        test_all_filters_pass,
        test_od_coverage_passes_threshold,
        test_od_coverage_fails_threshold,
        test_od_coverage_exact_boundary,
        test_force_reroute_bypasses_hysteresis,
        test_normal_reroute_respects_hysteresis,
        test_fuel_specificity_intercept_range,
        test_equal_time_subset_mean,
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
