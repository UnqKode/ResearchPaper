import os
import sys
from unittest.mock import MagicMock

# Force network-TraCI path in traci_compat so the stubbed 'traci' module is used
# (rather than attempting to import libsumo).
os.environ["FORCE_TRACI"] = "1"

# Stub dependencies
sys.modules['traci'] = MagicMock()
sys.modules['traci.exceptions'] = MagicMock()
sys.modules['traci.constants'] = MagicMock()
sys.modules['sumolib'] = MagicMock()
sys.modules['networkx'] = MagicMock()

from Simulation.simulate import Simulation
from traci_compat import traci

def test_persistence():
    sim = Simulation(net_file="dummy.net.xml")
    sim.ego_routing = "ours"
    sim.reroute_interval = 1

    # Time sequence:
    # 21600.0 : Injection (fake_inject called)
    # 21600.5 : Step 1 (current_step = 43201) -> evaluates and reroutes
    # 21601.0 : Step 2 (current_step = 43202) -> evaluates and reroutes again
    time_seq = [21600.0, 21600.5, 21601.0]
    def mock_getTime():
        if time_seq: return time_seq.pop(0)
        # Force timeout or completion by returning a large time
        return 21610.0
    traci.simulation.getTime.side_effect = mock_getTime
    traci.simulation.getMinExpectedNumber.return_value = 1
    traci.simulation.getDeltaT.return_value = 0.5

    sim.net_builder = MagicMock()
    sim.global_map = MagicMock()
    sim._safe_remove_ego = MagicMock()

    NEW_SNAPSHOT = {"route": ["E1", "E2"], "saved_weights": {"E1": 1.0, "E2": 2.0}, "timestamp": 5.0}
    EVAL_SNAPSHOT = {"route": ["E3"], "saved_weights": {"E3": 3.0}, "timestamp": 6.0}

    eval_calls = 0

    # 1. During injection, we reassign the snapshot
    def fake_inject(origin, dest, k):
        sim.route_snapshot = NEW_SNAPSHOT.copy()
        sim.ego_metrics["reroutes"] = 0
        return True
    sim._inject_ego_trip = fake_inject

    # 2. During evaluation, we assert the snapshot matches what was saved,
    # and then reassign it AGAIN to test the loop's save-back.
    def fake_evaluate():
        nonlocal eval_calls
        eval_calls += 1
        
        if eval_calls == 1:
            # First time evaluate is called, it should have the injected snapshot
            assert sim.route_snapshot == NEW_SNAPSHOT, "Snapshot did not persist from _inject_ego_trip"
            sim.route_snapshot = EVAL_SNAPSHOT.copy()
        elif eval_calls == 2:
            # Second time evaluate is called, it should have the snapshot from the first evaluate
            assert sim.route_snapshot == EVAL_SNAPSHOT, "Snapshot did not persist from _evaluate_and_reroute"
            
        sim.ego_metrics["reroutes"] += 1
    sim._evaluate_and_reroute = fake_evaluate

    def fake_track(dt):
        if eval_calls >= 2:
            sim.ego_metrics["arrive_time"] = 21601.5
    sim._track_ego = fake_track

    od_list = [("A", "B")]
    try:
        results = sim.run_fixed_departure_campaign(
            od_list=od_list,
            depart_start=21600,
            depart_spacing=300,
            per_trip_timeout=5,
            ego_policy="ours",
            reroute_interval=1,
            use_hysteresis=True
        )
    except StopIteration:
        pass

    assert eval_calls >= 2, "Evaluations did not run"
    print("Snapshot persistence verified successfully!")

if __name__ == "__main__":
    test_persistence()
