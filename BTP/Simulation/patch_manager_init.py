import os
import re

file_path = "compare_routing.py"
with open(file_path, "r") as f:
    content = f.read()

# Fix select_degraded_edges
old_sel = '''    net = sumolib.net.readNet(net_file)
    
    traci.start(sumo_cmd, port=port)
    rsu_manager = RSUManager(record_interval=300)'''
new_sel = '''    net = sumolib.net.readNet(net_file)
    
    traci.start(sumo_cmd, port=port)
    from Simulation.simulate import Simulation
    sim = Simulation(net_file)
    rsu_manager = sim.rsu_manager'''
content = content.replace(old_sel, new_sel)

# Fix Gate A verification
old_gate = '''        traci.start(sumo_cmd, port=port)
        rsu_manager = RSUManager(record_interval=300)
        from RSU.edgecost import EdgeCostCalculator
        calc = EdgeCostCalculator(alpha=1.0, beta=1.0, gamma=1.0)
        rsu_manager.set_edge_cost_calc(calc)'''
new_gate = '''        traci.start(sumo_cmd, port=port)
        from Simulation.simulate import Simulation
        sim = Simulation(NET_FILE)
        rsu_manager = sim.rsu_manager
        calc = sim.calc'''
content = content.replace(old_gate, new_gate)

with open(file_path, "w") as f:
    f.write(content)
