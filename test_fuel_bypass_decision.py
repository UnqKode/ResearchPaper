"""
Unit tests for the Step 2 fuel-vs-time bypass decision rule.

The rule (from _evaluate_and_reroute):
    fuel_ok = fuel_bypass <= fuel_through * (1 - theta_fuel)
    time_ok = time_bypass <= time_through * (1 + theta_time)
    if fuel_ok and time_ok: take bypass

Both conditions must be met.  theta_fuel=0.05, theta_time=0.10 are the
declared defaults for the production run (passed via --theta-fuel / --theta-time).
"""


def _decide(fuel_through, time_through, fuel_bypass, time_bypass,
            theta_fuel=0.05, theta_time=0.10):
    """Pure-logic mirror of the decision used in _evaluate_and_reroute."""
    if fuel_through <= 0:
        return False, False, False
    fuel_ok = fuel_bypass <= fuel_through * (1.0 - theta_fuel)
    time_ok = time_bypass <= time_through * (1.0 + theta_time)
    return fuel_ok, time_ok, fuel_ok and time_ok


def test_bypass_accepted_both_conditions_met():
    ok_f, ok_t, take = _decide(
        fuel_through=1000, time_through=100,
        fuel_bypass=900,   time_bypass=105,   # -10% fuel, +5% time
    )
    assert ok_f, "fuel condition should pass (-10% < -5% threshold)"
    assert ok_t, "time condition should pass (+5% < +10% limit)"
    assert take


def test_bypass_rejected_time_too_long():
    ok_f, ok_t, take = _decide(
        fuel_through=1000, time_through=100,
        fuel_bypass=900,   time_bypass=120,   # -10% fuel, +20% time
    )
    assert ok_f
    assert not ok_t, "time condition should fail (+20% > +10% limit)"
    assert not take


def test_bypass_rejected_fuel_saving_insufficient():
    ok_f, ok_t, take = _decide(
        fuel_through=1000, time_through=100,
        fuel_bypass=960,   time_bypass=105,   # -4% fuel (< 5% required), +5% time
    )
    assert not ok_f, "fuel condition should fail (-4% < -5% threshold)"
    assert ok_t
    assert not take


def test_bypass_rejected_bypass_uses_more_fuel():
    ok_f, ok_t, take = _decide(
        fuel_through=1000, time_through=100,
        fuel_bypass=1050,  time_bypass=105,   # +5% fuel, +5% time
    )
    assert not ok_f
    assert not take


def test_boundary_exactly_at_threshold_accepted():
    fuel_through, time_through = 1000.0, 100.0
    theta_fuel, theta_time = 0.05, 0.10
    fuel_bypass = fuel_through * (1.0 - theta_fuel)   # = 950.0
    time_bypass  = time_through * (1.0 + theta_time)  # = 110.0
    ok_f, ok_t, take = _decide(
        fuel_through, time_through, fuel_bypass, time_bypass,
        theta_fuel=theta_fuel, theta_time=theta_time,
    )
    assert ok_f, "exactly at fuel threshold should be accepted (<=)"
    assert ok_t, "exactly at time threshold should be accepted (<=)"
    assert take


def test_boundary_one_unit_over_fuel_threshold_rejected():
    fuel_through, time_through = 1000.0, 100.0
    theta_fuel, theta_time = 0.05, 0.10
    fuel_bypass = fuel_through * (1.0 - theta_fuel) + 0.001  # one epsilon over 950
    time_bypass  = time_through * (1.0 + theta_time)
    ok_f, ok_t, take = _decide(
        fuel_through, time_through, fuel_bypass, time_bypass,
        theta_fuel=theta_fuel, theta_time=theta_time,
    )
    assert not ok_f
    assert not take


def test_theta_fuel_1_disables_rule():
    # In production code the guard `if self.theta_fuel < 1.0` means the bypass path
    # is never computed at all.  At the pure-logic level, theta_fuel=1.0 requires
    # 100% fuel saving (fuel_bypass <= 0) which is impossible for any real bypass route.
    ok_f, ok_t, take = _decide(
        fuel_through=1000, time_through=100,
        fuel_bypass=500,   time_bypass=105,   # 50% saving — still < 100% required
        theta_fuel=1.0,    theta_time=0.10,
    )
    assert not ok_f, "theta_fuel=1.0 requires 100% fuel saving; 50% is insufficient"
    assert not take


def test_zero_fuel_through_does_not_crash():
    ok_f, ok_t, take = _decide(
        fuel_through=0, time_through=100,
        fuel_bypass=0,  time_bypass=105,
    )
    assert not take, "no decision when fuel_through=0"


if __name__ == "__main__":
    tests = [
        test_bypass_accepted_both_conditions_met,
        test_bypass_rejected_time_too_long,
        test_bypass_rejected_fuel_saving_insufficient,
        test_bypass_rejected_bypass_uses_more_fuel,
        test_boundary_exactly_at_threshold_accepted,
        test_boundary_one_unit_over_fuel_threshold_rejected,
        test_theta_fuel_1_disables_rule,
        test_zero_fuel_through_does_not_crash,
    ]
    passed = 0
    for fn in tests:
        try:
            fn()
            print(f"  PASS  {fn.__name__}")
            passed += 1
        except AssertionError as e:
            print(f"  FAIL  {fn.__name__}: {e}")
    print(f"\n{passed}/{len(tests)} tests passed")
    if passed < len(tests):
        raise SystemExit(1)
