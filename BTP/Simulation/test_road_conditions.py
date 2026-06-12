import unittest
from collections import defaultdict
import sys
import os

# Ensure we can import the module
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from road_conditions import RoadConditionManager

class MockTraCIEdge:
    def __init__(self):
        self.speeds = {}
        self.max_speeds = {"e1": 13.89, "e2": 13.89}
        self.lanes = {"e1": 2, "e2": 1}
        self.set_calls = 0

    def getMaxSpeed(self, edge):
        return self.max_speeds.get(edge, 13.89)

    def setMaxSpeed(self, edge, speed):
        self.speeds[edge] = speed
        self.set_calls += 1

    def getLaneNumber(self, edge):
        return self.lanes.get(edge, 1)

class MockTraCILane:
    def __init__(self):
        self.permissions = {}
        self.disallowed = defaultdict(list)
        self.set_calls = 0

    def getAllowed(self, lane):
        return self.permissions.get(lane, ["passenger", "bus"])

    def setAllowed(self, lane, allowed):
        self.permissions[lane] = allowed
        self.set_calls += 1

    def setDisallowed(self, lane, disallowed):
        self.disallowed[lane] = disallowed
        self.set_calls += 1

class MockTraCI:
    def __init__(self):
        self.edge = MockTraCIEdge()
        self.lane = MockTraCILane()

class MockCalc:
    def __init__(self, locked_edges):
        self._fuel_baseline = {e: 100 for e in locked_edges}

class TestRoadConditionManager(unittest.TestCase):
    def setUp(self):
        self.traci = MockTraCI()

    def test_rough_mode_activation_and_oscillation(self):
        # 1. Deterministic construction
        mgr = RoadConditionManager(["e1", "e2"], "rough", activate_time=100.0, v_low=5.0, v_high=12.0, period_s=20.0)
        mgr.bind(self.traci, MockCalc(["e1", "e2"]))
        
        # 2. No TraCI calls before activate_time
        mgr.step(50.0)
        self.assertEqual(self.traci.edge.set_calls, 0)
        
        # 3. Activation at exactly activate_time
        mgr.step(100.0)
        self.assertTrue(mgr.active)
        self.assertEqual(self.traci.edge.set_calls, 2)  # Initial drop for both edges
        self.assertEqual(self.traci.edge.speeds["e1"], 5.0)
        self.assertEqual(self.traci.edge.speeds["e2"], 5.0)
        
        # 4. No extra calls until half period passes
        mgr.step(105.0)
        self.assertEqual(self.traci.edge.set_calls, 2)
        
        # 5. Oscillation toggles at period/2
        mgr.step(110.0)
        self.assertEqual(self.traci.edge.set_calls, 4)
        self.assertEqual(self.traci.edge.speeds["e1"], 12.0)
        self.assertEqual(self.traci.edge.speeds["e2"], 12.0)
        
        # 6. Deactivation restores original speeds
        mgr.deactivate()
        self.assertFalse(mgr.active)
        self.assertTrue(mgr.restored)
        self.assertEqual(self.traci.edge.set_calls, 6)
        self.assertEqual(self.traci.edge.speeds["e1"], 13.89)

    def test_baseline_lock_warning(self):
        # Only e1 is locked in calc
        mgr = RoadConditionManager(["e1", "e2"], "rough", activate_time=100.0)
        mgr.bind(self.traci, MockCalc(["e1"]))
        
        with self.assertLogs(level='WARNING') as log:
            mgr.step(100.0)
            self.assertTrue(any("UNLOCKED" in m for m in log.output))

    def test_accident_mode(self):
        mgr = RoadConditionManager(["e1"], "accident", activate_time=100.0, event_duration=600.0)
        mgr.bind(self.traci, MockCalc(["e1"]))
        
        # Activate
        mgr.step(100.0)
        self.assertTrue(mgr.active)
        self.assertEqual(self.traci.lane.set_calls, 1)
        self.assertIn("passenger", self.traci.lane.disallowed["e1_0"])
        
        # Step within duration - no extra calls
        mgr.step(400.0)
        self.assertEqual(self.traci.lane.set_calls, 1)
        
        # Step after duration - auto restores
        mgr.step(700.0)
        self.assertFalse(mgr.active)
        self.assertTrue(mgr.restored)
        self.assertEqual(self.traci.lane.set_calls, 2)

if __name__ == '__main__':
    unittest.main()
