import os
import sys
import sumolib
import traci

sys.path.append(os.path.abspath("."))
from RSU.rsu import RSUManager
from RSU.edgecost import EdgeCostCalculator

def run_calibration(net_file, end_time, traffic_seed, scale, teleport):
    print("Running headless calibration to select degraded edges...")
    cmd = [
        "sumo", "-c", "scenario/in/most.commercial.sumocfg",
        "--seed", str(traffic_seed),
        "--scale", str(scale),
        "--no-step-log", "true",
        "--no-warnings", "true",
        "--time-to-teleport", str(teleport),
    ]
    traci.start(cmd)
    
    rsu_manager = RSUManager(record_interval=300)
    calc = EdgeCostCalculator(alpha=1.0, beta=1.0, gamma=1.0)
    rsu_manager.set_edge_cost_calc(calc)
    
    while traci.simulation.getTime() < end_time:
        traci.simulationStep()
        rsu_manager.step()
        
    # Get edge stats
    edges = traci.edge.getIDList()
    edge_counts = []
    for e in edges:
        if not e.startswith(":"):
            stats = rsu_manager.get_edge_stats(e)
            if stats:
                edge_counts.append((e, stats.get("vehicle_count", 0)))
                
    traci.close()
    
    edge_counts.sort(key=lambda x: x[1], reverse=True)
    return edge_counts[:10]

counts = run_calibration("scenario/in/most.net.xml", 21300, 1, 2.0, 300)
print(counts)
