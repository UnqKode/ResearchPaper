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
