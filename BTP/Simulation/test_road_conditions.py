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
        self._edge_vehicles = {}   # edge_id -> list of vids

    def getLaneNumber(self, edge):
        return self.lane_counts.get(edge, 1)

    def getLastStepVehicleIDs(self, edge):
        return self._edge_vehicles.get(edge, [])


class MockTraCIVehicle:
    def __init__(self):
        self._emission_classes = {}  # vid -> current emission class
        self.set_class_log = []      # (vid, cls) log

    def getEmissionClass(self, vid):
        return self._emission_classes.get(vid, "HBEFA3/PC_G_EU6")

    def setEmissionClass(self, vid, cls):
        self._emission_classes[vid] = cls
        self.set_class_log.append((vid, cls))

    def getIDList(self):
        return list(self._emission_classes.keys())


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
        self.vehicle = MockTraCIVehicle()


class MockCalc:
    def __init__(self, locked_edges):
        self._fuel_baseline = {e: 100 for e in locked_edges}
        self.frozen = set()

    def freeze_baseline(self, edge_id):
        self.frozen.add(edge_id)

    def unfreeze_baseline(self, edge_id):
        self.frozen.discard(edge_id)


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


    # ------------------------------------------------------------------
    # 6. Grade mode: emission class swapped on entry, restored on exit
    # ------------------------------------------------------------------
    def test_grade_mode_swap_on_entry(self):
        mgr = RoadConditionManager(
            ["e1"], "grade", activate_time=100.0,
            grade_emission_class="HBEFA3/PC_G_EU0"
        )
        mgr.bind(self.traci, MockCalc(["e1"]))

        # Place a vehicle on the degraded edge
        self.traci.edge._edge_vehicles["e1"] = ["v1"]
        self.traci.vehicle._emission_classes["v1"] = "HBEFA3/PC_G_EU6"

        # Activate and run one step
        mgr.step(100.0)
        self.assertTrue(mgr.active)
        self.assertEqual(self.traci.vehicle._emission_classes["v1"], "HBEFA3/PC_G_EU0",
                         "Emission class must be swapped to heavy class on entry")
        self.assertIn("v1", mgr._grade_modified_vehicles)
        self.assertEqual(mgr._grade_modified_vehicles["v1"], "HBEFA3/PC_G_EU6",
                         "Original class must be stored for later restoration")

    def test_grade_mode_restore_on_exit(self):
        mgr = RoadConditionManager(
            ["e1"], "grade", activate_time=100.0,
            grade_emission_class="HBEFA3/PC_G_EU0"
        )
        mgr.bind(self.traci, MockCalc(["e1"]))

        # Vehicle enters the degraded edge
        self.traci.edge._edge_vehicles["e1"] = ["v1"]
        self.traci.vehicle._emission_classes["v1"] = "HBEFA3/PC_G_EU6"
        mgr.step(100.0)
        self.assertEqual(self.traci.vehicle._emission_classes["v1"], "HBEFA3/PC_G_EU0")

        # Vehicle leaves the degraded edge (not on any degraded edge anymore)
        self.traci.edge._edge_vehicles["e1"] = []
        mgr.step(101.0)
        self.assertEqual(self.traci.vehicle._emission_classes["v1"], "HBEFA3/PC_G_EU6",
                         "Emission class must be restored when vehicle exits degraded edge")
        self.assertNotIn("v1", mgr._grade_modified_vehicles)

    def test_grade_mode_deactivate_restores_all(self):
        mgr = RoadConditionManager(
            ["e1"], "grade", activate_time=100.0,
            grade_emission_class="HBEFA3/PC_G_EU0"
        )
        mgr.bind(self.traci, MockCalc(["e1"]))

        # Two vehicles on the degraded edge
        self.traci.edge._edge_vehicles["e1"] = ["v1", "v2"]
        for vid in ["v1", "v2"]:
            self.traci.vehicle._emission_classes[vid] = "HBEFA3/PC_G_EU6"
        mgr.step(100.0)
        self.assertEqual(self.traci.vehicle._emission_classes["v1"], "HBEFA3/PC_G_EU0")
        self.assertEqual(self.traci.vehicle._emission_classes["v2"], "HBEFA3/PC_G_EU0")

        # Deactivate while both vehicles are still on the edge
        mgr.deactivate()
        self.assertFalse(mgr.active)
        self.assertTrue(mgr.restored)
        self.assertEqual(self.traci.vehicle._emission_classes["v1"], "HBEFA3/PC_G_EU6",
                         "v1 class must be restored on deactivate")
        self.assertEqual(self.traci.vehicle._emission_classes["v2"], "HBEFA3/PC_G_EU6",
                         "v2 class must be restored on deactivate")
        self.assertEqual(len(mgr._grade_modified_vehicles), 0,
                         "_grade_modified_vehicles must be empty after deactivate")

    def test_grade_mode_no_speed_change(self):
        """Grade mode must NOT alter speed limits."""
        mgr = RoadConditionManager(
            ["e1", "e2"], "grade", activate_time=100.0,
            grade_emission_class="HBEFA3/PC_G_EU0"
        )
        mgr.bind(self.traci, MockCalc(["e1", "e2"]))
        mgr.step(100.0)
        self.assertTrue(mgr.active)
        self.assertEqual(self.traci.lane._speed_set_calls, 0,
                         "Grade mode must not call setMaxSpeed")

    def test_grade_mode_freezes_and_unfreezes_baseline(self):
        """Activation must freeze baselines; deactivation must unfreeze them."""
        calc = MockCalc(["e1", "e2"])
        mgr = RoadConditionManager(
            ["e1", "e2"], "grade", activate_time=100.0,
            grade_emission_class="HBEFA3/PC_G_EU0"
        )
        mgr.bind(self.traci, calc)

        # Before activation: no edges frozen
        self.assertEqual(calc.frozen, set())

        mgr.step(100.0)
        self.assertTrue(mgr.active)
        self.assertIn("e1", calc.frozen, "e1 baseline must be frozen on grade activation")
        self.assertIn("e2", calc.frozen, "e2 baseline must be frozen on grade activation")

        mgr.deactivate()
        self.assertFalse(mgr.active)
        self.assertNotIn("e1", calc.frozen, "e1 baseline must be unfrozen after deactivation")
        self.assertNotIn("e2", calc.frozen, "e2 baseline must be unfrozen after deactivation")


if __name__ == "__main__":
    unittest.main()
