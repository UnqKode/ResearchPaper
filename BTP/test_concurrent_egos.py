import sys
import unittest
from unittest.mock import MagicMock, patch

import traci
from Simulation.simulate import Simulation

class TestConcurrentEgos(unittest.TestCase):
    @patch('Simulation.simulate.traci')
    def test_concurrent_ego_tracking(self, mock_traci):
        # Setup mock behavior
        mock_traci.simulation.getDeltaT.return_value = 0.25
        mock_traci.simulation.getTime.side_effect = lambda: float(TestConcurrentEgos.current_step) * 0.25
        mock_traci.simulation.getMinExpectedNumber.return_value = 100
        
        # Simulate arrive sequence
        mock_traci.simulation.getArrivedIDList.side_effect = lambda: TestConcurrentEgos.arrived_list
        mock_traci.vehicle.getDistance.return_value = 1000.0
        # Mock NetworkBuilder to bypass actual parsing
        with patch('Simulation.simulate.NetworkBuilder') as mock_nb:
            mock_nb_instance = MagicMock()
            mock_nb_instance.get_graph.return_value = MagicMock()
            mock_nb_instance.get_intersections.return_value = []
            mock_nb_instance.get_all_edges.return_value = []
            mock_nb_instance.net.getEdges.return_value = []
            mock_nb.return_value = mock_nb_instance
            
            sim = Simulation(
                net_file="dummy.net.xml",
                reroute_interval=10,
                ego_vehicle_id="ego_test",
                ego_origin="orig",
                ego_dest="dest",
                ego_type="DEFAULT_VEHTYPE",
                ego_depart=0,
                ego_routing="ours"
            )
        
        sim.global_map = MagicMock()
        sim.net_builder = MagicMock()
        sim.rsu_manager = MagicMock()
        
        # Stub the injection to just succeed
        def mock_inject(*args, **kwargs):
            return True
        sim._inject_ego_trip = mock_inject
        
        # We will manually step through run_fixed_departure_campaign using a small OD list
        od_list = [("A", "B"), ("C", "D")]
        depart_start = 10.0
        depart_spacing = 5.0
        
        # Prepare the state just before the loop
        schedule = {}
        ego_states = {}
        for k, (origin, dest) in enumerate(od_list):
            depart_time_s = depart_start + k * depart_spacing
            vid = f"ego_{k}"
            schedule[vid] = depart_time_s
            ego_states[vid] = {
                "metrics": {"depart_time": None, "arrive_time": None, "fuel_mg": 0.0, "reroutes": 0},
                "snapshot": {"route": [], "saved_weights": {}, "timestamp": 0.0},
                "last_dijkstra_time": 0.0,
                "origin": origin,
                "dest": dest,
                "depart_t": None,
            }
            
        active_egos = set()
        completed_egos = set()
        results = []
        
        # Step 0: time = 10.0s (step=40). Inject ego_0.
        TestConcurrentEgos.current_step = 40
        sim_time = 10.0
        TestConcurrentEgos.arrived_list = []
        
        # Run inject logic for step 0
        vid = "ego_0"
        self.assertTrue(sim_time >= schedule[vid])
        
        sim.ego_id = vid
        sim.ego_metrics = ego_states[vid]["metrics"]
        sim.route_snapshot = ego_states[vid]["snapshot"]
        sim.last_dijkstra_time = ego_states[vid]["last_dijkstra_time"]
        ego_states[vid]["depart_t"] = sim_time
        
        sim._inject_ego_trip(od_list[0][0], od_list[0][1], 0)
        active_egos.add(vid)
        
        # Track loop for step 0
        for v in list(active_egos):
            sim.ego_id = v
            sim.ego_metrics = ego_states[v]["metrics"]
            sim.route_snapshot = ego_states[v]["snapshot"]
            sim.last_dijkstra_time = ego_states[v]["last_dijkstra_time"]
            sim._track_ego(0.25)
            # Reroute logic
            sim.ego_metrics["reroutes"] += 1
            ego_states[v]["last_dijkstra_time"] = sim.last_dijkstra_time

        self.assertEqual(len(active_egos), 1)
        self.assertIsNone(ego_states["ego_0"]["metrics"]["arrive_time"])
        self.assertEqual(ego_states["ego_0"]["metrics"]["reroutes"], 1)
        
        # Step 20: time = 15.0s (step=60). Inject ego_1. ego_0 is still active.
        TestConcurrentEgos.current_step = 60
        sim_time = 15.0
        
        vid = "ego_1"
        self.assertTrue(sim_time >= schedule[vid])
        sim.ego_id = vid
        sim.ego_metrics = ego_states[vid]["metrics"]
        sim.route_snapshot = ego_states[vid]["snapshot"]
        sim.last_dijkstra_time = ego_states[vid]["last_dijkstra_time"]
        ego_states[vid]["depart_t"] = sim_time
        
        sim._inject_ego_trip(od_list[1][0], od_list[1][1], 1)
        active_egos.add(vid)
        
        # ego_0 arrives at this step!
        TestConcurrentEgos.arrived_list = ["ego_0"]
        
        for v in list(active_egos):
            sim.ego_id = v
            sim.ego_metrics = ego_states[v]["metrics"]
            sim.route_snapshot = ego_states[v]["snapshot"]
            sim.last_dijkstra_time = ego_states[v]["last_dijkstra_time"]
            sim._track_ego(0.25)
            # Reroute logic
            sim.ego_metrics["reroutes"] += 1
            ego_states[v]["last_dijkstra_time"] = sim.last_dijkstra_time

        # ego_0 should have arrived, ego_1 should not have
        self.assertIsNotNone(ego_states["ego_0"]["metrics"]["arrive_time"])
        self.assertIsNone(ego_states["ego_1"]["metrics"]["arrive_time"])
        
        # Remove arrived
        for v in list(active_egos):
            if ego_states[v]["metrics"]["arrive_time"] is not None:
                active_egos.remove(v)
                completed_egos.add(v)
                
        # ego_1 remains active and is still rerouted
        self.assertEqual(len(active_egos), 1)
        self.assertIn("ego_1", active_egos)
        self.assertEqual(ego_states["ego_1"]["metrics"]["reroutes"], 1)
        
        # Step 40: time = 20.0s (step=80). ego_1 is active and ego_0 is complete.
        TestConcurrentEgos.current_step = 80
        sim_time = 20.0
        TestConcurrentEgos.arrived_list = []
        
        for v in list(active_egos):
            sim.ego_id = v
            sim.ego_metrics = ego_states[v]["metrics"]
            sim.route_snapshot = ego_states[v]["snapshot"]
            sim.last_dijkstra_time = ego_states[v]["last_dijkstra_time"]
            sim._track_ego(0.25)
            # Reroute logic
            sim.ego_metrics["reroutes"] += 1
            ego_states[v]["last_dijkstra_time"] = sim.last_dijkstra_time
            
        self.assertEqual(ego_states["ego_0"]["metrics"]["reroutes"], 2) # Stopped rerouting
        self.assertEqual(ego_states["ego_1"]["metrics"]["reroutes"], 2) # Continues to reroute

if __name__ == "__main__":
    unittest.main()
