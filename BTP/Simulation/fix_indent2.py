import os
import re

file_path = "compare_routing.py"
with open(file_path, "r") as f:
    content = f.read()

old_block = '''        from Simulation.road_conditions import RoadConditionManager
        manager = RoadConditionManager(
            degraded_edges, args.road_condition, degrade_start,
            v_low=args.degrade_vlow, v_high=args.degrade_vhigh,
            period_s=args.degrade_period, event_duration=args.event_duration
        )
    if args.paired:'''
new_block = '''    from Simulation.road_conditions import RoadConditionManager
    manager = RoadConditionManager(
        degraded_edges, args.road_condition, degrade_start,
        v_low=args.degrade_vlow, v_high=args.degrade_vhigh,
        period_s=args.degrade_period, event_duration=args.event_duration
    )
    if args.paired:'''
content = content.replace(old_block, new_block)

with open(file_path, "w") as f:
    f.write(content)
