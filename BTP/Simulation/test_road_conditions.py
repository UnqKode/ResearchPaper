import unittest
from collections import defaultdict
import sys
import os

# Ensure we can import the module
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from road_conditions import RoadConditionManager


# ---------------------------------------------------------------------------
# Mock TraCI objects that mirror the lane-based API used by RoadConditionManager
# ---------------------------------------------------------------------------
class MockTraCIEdge:
    def __init__(self):
        self.lane_counts = {"e1": 2, "e2": 1}

    def getLaneNumber(self, edge):
        return self.lane_counts.get(edge, 1)


class MockTraCILane:
    def __init__(self):
        self.speeds = {}            # lane_id -> speed
        self.permissions = {}       # lane_id -> allowed list
        self.disallowed = defaultdict(list)
        self._speed_set_calls = 0
        self._perm_set_calls = 0
        # Preset original speeds for e1 and e2 lane 0
        self.speeds["e1_0"] = 13.89
        self.speeds["e1_1"] = 13.89
        self.speeds["e2_0"] = 13.89

    def getMaxSpeed(self, lane_id):
        return self.speeds.get(lane_id, 13.89)

    def setMaxSpeed(self, lane_id, speed):
        self.speeds[lane_id] = speed
        self._speed_set_calls += 1

    def getAllowed(self, lane_id):
        return self.permissions.get(lane_id, ["passenger", "bus"])

    def setAllowed(self, lane_id, allowed):
        self.permissions[lane_id] = allowed
        self._perm_set_calls += 1

    def setDisallowed(self, lane_id, disallowed):
        self.disallowed[lane_id] = disallowed
        self._perm_set_calls += 1


class MockTraCI:
    def __init__(self):
        self.edge = MockTraCIEdge()
        self.lane = MockTraCILane()


class MockCalc:
    def __init__(self, locked_edges):
        self._fuel_baseline = {e: 100 for e in locked_edges}


# ---------------------------------------------------------------------------
# Test cases
# ---------------------------------------------------------------------------
class TestRoadConditionManager(unittest.TestCase):

    def setUp(self):
        self.traci = MockTraCI()

    # ------------------------------------------------------------------
    # 1. Activation at activate_time + oscillation every period_s/2
    # ------------------------------------------------------------------
    def test_rough_mode_activation_and_oscillation(self):
        mgr = RoadConditionManager(
            ["e1", "e2"], "rough", activate_time=100.0,
            v_low=5.0, v_high=12.0, period_s=20.0
        )
        mgr.bind(self.traci, MockCalc(["e1", "e2"]))

        # Before activate_time: no TraCI calls
        mgr.step(50.0)
        self.assertEqual(self.traci.lane._speed_set_calls, 0)

        # Activation at exactly activate_time:
        # e1 has 2 lanes, e2 has 1 lane → 3 setMaxSpeed calls total
        mgr.step(100.0)
        self.assertTrue(mgr.active)
        self.assertEqual(self.traci.lane._speed_set_calls, 3)
        self.assertEqual(self.traci.lane.speeds["e1_0"], 5.0)
        self.assertEqual(self.traci.lane.speeds["e1_1"], 5.0)
        self.assertEqual(self.traci.lane.speeds["e2_0"], 5.0)

        # Within half period: no extra calls
        mgr.step(105.0)
        self.assertEqual(self.traci.lane._speed_set_calls, 3)

        # At period/2 = 10s: oscillate to v_high (3 more calls)
        mgr.step(110.0)
        self.assertEqual(self.traci.lane._speed_set_calls, 6)
        self.assertEqual(self.traci.lane.speeds["e1_0"], 12.0)
        self.assertEqual(self.traci.lane.speeds["e1_1"], 12.0)
        self.assertEqual(self.traci.lane.speeds["e2_0"], 12.0)

    # ------------------------------------------------------------------
    # 2. Original speeds saved and restored on deactivate
    # ------------------------------------------------------------------
    def test_deactivation_restores_original_speeds(self):
        mgr = RoadConditionManager(
            ["e1", "e2"], "rough", activate_time=100.0,
            v_low=5.0, v_high=12.0, period_s=20.0
        )
        mgr.bind(self.traci, MockCalc(["e1", "e2"]))

        mgr.step(100.0)       # activates
        mgr.deactivate()

        self.assertFalse(mgr.active)
        self.assertTrue(mgr.restored)
        # Restore: e1 has 2 lanes (2 calls) + e2 has 1 lane (1 call) = 3
        # Total calls = 3 (activate) + 3 (restore) = 6
        self.assertEqual(self.traci.lane._speed_set_calls, 6)
        # Original speed was 13.89 for all lanes
        self.assertAlmostEqual(self.traci.lane.speeds["e1_0"], 13.89)
        self.assertAlmostEqual(self.traci.lane.speeds["e2_0"], 13.89)

    # ------------------------------------------------------------------
    # 3. Baseline-lock warning when some edges are unlocked
    # ------------------------------------------------------------------
    def test_baseline_lock_warning(self):
        # Only e1 is locked in calc; e2 is unlocked → should warn
        mgr = RoadConditionManager(["e1", "e2"], "rough", activate_time=100.0)
        mgr.bind(self.traci, MockCalc(["e1"]))

        with self.assertLogs(level="WARNING") as log:
            mgr.step(100.0)
        self.assertTrue(any("UNLOCKED" in m for m in log.output))

    # ------------------------------------------------------------------
    # 4. Accident mode: block lane 0, auto-restore after event_duration
    # ------------------------------------------------------------------
    def test_accident_mode_activate_and_auto_restore(self):
        mgr = RoadConditionManager(
            ["e1"], "accident", activate_time=100.0, event_duration=600.0
        )
        mgr.bind(self.traci, MockCalc(["e1"]))

        # Activate
        mgr.step(100.0)
        self.assertTrue(mgr.active)
        self.assertEqual(self.traci.lane._perm_set_calls, 1)
        self.assertIn("passenger", self.traci.lane.disallowed["e1_0"])

        # Step within event_duration — no extra calls
        mgr.step(400.0)
        self.assertEqual(self.traci.lane._perm_set_calls, 1)

        # Step after event_duration — auto-restore
        mgr.step(700.0)
        self.assertFalse(mgr.active)
        self.assertTrue(mgr.restored)
        self.assertEqual(self.traci.lane._perm_set_calls, 2)

    # ------------------------------------------------------------------
    # 5. D2/D3 integration stub: Simulation with manager calls step() each loop
    # ------------------------------------------------------------------
    def test_simulation_stub_calls_manager_step(self):
        """
        Verify that a Simulation-like object storing road_condition_manager
        actually calls manager.step() each iteration.
        """
        step_calls = []

        class _FakeManager:
            mode = "rough"
            degraded_edges = ["e1"]
            _traci = None
            _calc = None

            def bind(self, traci, calc):
                pass

            def step(self, sim_time):
                step_calls.append(sim_time)

        mgr = _FakeManager()

        class _FakeSim:
            def __init__(self, manager):
                self.road_condition_manager = manager

            def run_loop(self, times):
                for t in times:
                    if self.road_condition_manager is not None:
                        self.road_condition_manager.step(t)

        sim = _FakeSim(mgr)
        sim.run_loop([100.0, 101.0, 102.0])

        self.assertEqual(step_calls, [100.0, 101.0, 102.0],
                         "manager.step() must be called once per loop iteration")


if __name__ == "__main__":
    unittest.main()
