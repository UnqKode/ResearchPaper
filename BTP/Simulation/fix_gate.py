import os
import re

file_path = "compare_routing.py"
with open(file_path, "r") as f:
    content = f.read()

# Fix subscribe_edges and stats key
old_gate = '''        from Simulation.simulate import Simulation
        sim = Simulation(NET_FILE)
        rsu_manager = sim.rsu_manager
        calc = sim.calc
        
        manager = RoadConditionManager(
            degraded_edges, args.road_condition, degrade_start,
            v_low=args.degrade_vlow, v_high=args.degrade_vhigh,
            period_s=args.degrade_period, event_duration=args.event_duration
        )
        manager.bind(traci, calc)
        
        fuel_before = {e: [] for e in degraded_edges}'''

new_gate = '''        from Simulation.simulate import Simulation
        sim = Simulation(NET_FILE)
        rsu_manager = sim.rsu_manager
        calc = sim.calc
        rsu_manager.subscribe_edges()
        
        manager = RoadConditionManager(
            degraded_edges, args.road_condition, degrade_start,
            v_low=args.degrade_vlow, v_high=args.degrade_vhigh,
            period_s=args.degrade_period, event_duration=args.event_duration
        )
        manager.bind(traci, calc)
        
        fuel_before = {e: [] for e in degraded_edges}'''

content = content.replace(old_gate, new_gate)

content = content.replace('f = stats.get("avg_fuel_mg", 0)', 'f = stats.get("fuel_consumption", 0)')

with open(file_path, "w") as f:
    f.write(content)
