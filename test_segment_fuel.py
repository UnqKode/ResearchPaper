"""
Unit tests for EdgeCostCalculator.get_segment_fuel and related fuel-objective
routing helpers.  All SUMO/TraCI-free.
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "BTP"))

from RSU.edgecost import EdgeCostCalculator


def _make_calc(fuel_window_size=10, nominal_fuel_rate_mg_s=50.0):
    return EdgeCostCalculator(
        edge_lengths={"e1": 200.0, "e2": 100.0},
        edge_speed_limits={"e1": 13.89, "e2": 13.89},
        fuel_window_size=fuel_window_size,
        nominal_fuel_rate_mg_s=nominal_fuel_rate_mg_s,
    )


# ── 1. Cold fallback is finite and positive ────────────────────────────────────
def test_cold_fallback_positive_finite():
    calc = _make_calc(nominal_fuel_rate_mg_s=50.0)
    v = calc.get_segment_fuel("e1")
    assert v > 0, "cold fallback must be positive"
    assert v < float("inf"), "cold fallback must be finite"


# ── 2. Cold fallback is NOT zero ───────────────────────────────────────────────
def test_cold_fallback_not_zero():
    calc = _make_calc()
    assert calc.get_segment_fuel("unknown_edge") != 0.0


# ── 3. Recency-weighted average: N samples, newest weighted highest ─────────────
def test_recency_weighted_average_newest_dominates():
    calc = _make_calc(fuel_window_size=10)
    # Feed 5 low samples then 1 high sample.  Weighted average must exceed simple mean.
    for _ in range(5):
        calc.record_traversal_fuel("e1", 100.0)
    calc.record_traversal_fuel("e1", 1000.0)   # newest, weight=6

    weighted = calc.get_segment_fuel("e1")
    simple_mean = (5 * 100.0 + 1000.0) / 6
    assert weighted > simple_mean, (
        f"recency weighting should make newest sample dominate: "
        f"weighted={weighted:.1f} vs simple_mean={simple_mean:.1f}"
    )


# ── 4. Exact recency-weighted average check ────────────────────────────────────
def test_recency_weighted_average_exact():
    calc = _make_calc(fuel_window_size=10)
    # 3 samples: [100, 200, 300] → weights [1, 2, 3], w_sum=6
    # expected = (100*1 + 200*2 + 300*3) / 6 = (100+400+900)/6 = 1400/6 ≈ 233.33
    for v in [100.0, 200.0, 300.0]:
        calc.record_traversal_fuel("e1", v)
    result = calc.get_segment_fuel("e1")
    expected = (100 * 1 + 200 * 2 + 300 * 3) / 6
    assert abs(result - expected) < 1e-9, f"expected {expected}, got {result}"


# ── 5. Window maxlen: only last N samples kept ─────────────────────────────────
def test_window_maxlen_keeps_last_n():
    calc = _make_calc(fuel_window_size=3)
    # Push 5 values; only the last 3 ([300, 400, 500]) should survive
    for v in [100.0, 200.0, 300.0, 400.0, 500.0]:
        calc.record_traversal_fuel("e1", v)
    # weights [1,2,3], w_sum=6
    expected = (300 * 1 + 400 * 2 + 500 * 3) / 6
    result = calc.get_segment_fuel("e1")
    assert abs(result - expected) < 1e-9, f"expected {expected:.2f}, got {result:.2f}"


# ── 6. Zero values are never stored ───────────────────────────────────────────
def test_zero_never_stored():
    calc = _make_calc(fuel_window_size=10)
    calc.record_traversal_fuel("e1", 0.0)
    calc.record_traversal_fuel("e1", -5.0)
    calc.record_traversal_fuel("e1", None)
    # Window should still be empty → cold fallback returned
    v = calc.get_segment_fuel("e1")
    assert v > 0
    assert "e1" not in calc._traversal_fuel or len(calc._traversal_fuel["e1"]) == 0


# ── 7. Recent high-fuel traversal moves average more than equal old one ─────────
def test_recent_high_fuel_moves_average_more():
    calc_recent = _make_calc(fuel_window_size=10)
    calc_old    = _make_calc(fuel_window_size=10)

    # Scenario A: [100, 100, 100, 100, 1000]  ← spike is NEWEST
    for _ in range(4):
        calc_recent.record_traversal_fuel("e1", 100.0)
    calc_recent.record_traversal_fuel("e1", 1000.0)

    # Scenario B: [1000, 100, 100, 100, 100]  ← spike is OLDEST
    calc_old.record_traversal_fuel("e1", 1000.0)
    for _ in range(4):
        calc_old.record_traversal_fuel("e1", 100.0)

    w_recent = calc_recent.get_segment_fuel("e1")
    w_old    = calc_old.get_segment_fuel("e1")
    assert w_recent > w_old, (
        f"spike at newest position should produce higher average: "
        f"recent={w_recent:.1f} vs old={w_old:.1f}"
    )


# ── 8. clear_traversal_fuel empties the window ────────────────────────────────
def test_clear_traversal_fuel():
    calc = _make_calc(fuel_window_size=10)
    for v in [200.0, 300.0, 400.0]:
        calc.record_traversal_fuel("e1", v)
    assert calc._traversal_fuel["e1"]  # non-empty before clear
    calc.clear_traversal_fuel(["e1"])
    assert len(calc._traversal_fuel["e1"]) == 0
    # get_segment_fuel should now return cold fallback (positive, finite)
    v = calc.get_segment_fuel("e1")
    assert v > 0 and v < float("inf")


# ── 9. reset() clears _traversal_fuel entirely ────────────────────────────────
def test_reset_clears_traversal_fuel():
    calc = _make_calc(fuel_window_size=10)
    calc.record_traversal_fuel("e1", 500.0)
    calc.reset()
    assert calc._traversal_fuel == {}, "reset() should clear _traversal_fuel"


# ── 10. Cold fallback uses EMA baseline when available ────────────────────────
def test_cold_fallback_uses_ema_baseline():
    calc = _make_calc(nominal_fuel_rate_mg_s=50.0)
    # Lock the baseline by feeding baseline_seed_n samples to e1
    for _ in range(calc._baseline_seed_n):
        calc.record_vehicle_fuel("e1", 80.0)   # rate (mg/s)
    assert calc._fuel_baseline.get("e1") is not None

    # Cold fallback for e1 should use 80 mg/s (the locked baseline), not 50
    L    = calc.edge_lengths["e1"]    # 200 m
    vlim = calc.edge_speed_limits["e1"]  # 13.89 m/s
    expected = 80.0 * (L / vlim)
    result = calc._cold_fuel_fallback("e1")
    assert abs(result - expected) < 1e-6, f"expected {expected:.2f}, got {result:.2f}"


# ── runner ────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    tests = [
        test_cold_fallback_positive_finite,
        test_cold_fallback_not_zero,
        test_recency_weighted_average_newest_dominates,
        test_recency_weighted_average_exact,
        test_window_maxlen_keeps_last_n,
        test_zero_never_stored,
        test_recent_high_fuel_moves_average_more,
        test_clear_traversal_fuel,
        test_reset_clears_traversal_fuel,
        test_cold_fallback_uses_ema_baseline,
    ]
    passed = 0
    for fn in tests:
        try:
            fn()
            print(f"  PASS  {fn.__name__}")
            passed += 1
        except AssertionError as e:
            print(f"  FAIL  {fn.__name__}: {e}")
        except Exception as e:
            print(f"  ERROR {fn.__name__}: {e}")
    print(f"\n{passed}/{len(tests)} tests passed")
    if passed < len(tests):
        raise SystemExit(1)
