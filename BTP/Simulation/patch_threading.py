import os
import re

file_path = "compare_routing.py"
with open(file_path, "r") as f:
    content = f.read()

# Replace run_paired_scenario_wrapper signature
old_wrapper_sig = "def run_paired_scenario_wrapper(arm_policy, alpha, beta, gamma, od_list, traffic_seed, tag,\n                                depart_start, depart_spacing, use_hysteresis, scale, teleport, debug_cfs=False):"
new_wrapper_sig = "def run_paired_scenario_wrapper(arm_policy, alpha, beta, gamma, od_list, traffic_seed, tag,\n                                depart_start, depart_spacing, use_hysteresis, scale, teleport, road_condition_manager=None, debug_cfs=False):"
content = content.replace(old_wrapper_sig, new_wrapper_sig)

# Replace the inner call in wrapper
old_inner = "res, cfs, route_cfs = run_paired_scenario(arm_policy, alpha, beta, gamma, od_list, traffic_seed, tag,\n                                      depart_start, depart_spacing, use_hysteresis, scale, teleport, debug_cfs=debug_cfs)"
new_inner = "res, cfs, route_cfs = run_paired_scenario(arm_policy, alpha, beta, gamma, od_list, traffic_seed, tag,\n                                      depart_start, depart_spacing, use_hysteresis, scale, teleport, road_condition_manager=road_condition_manager, debug_cfs=debug_cfs)"
content = content.replace(old_inner, new_inner)

# In main(), construct manager but do not bind it
main_manager = '''        from Simulation.road_conditions import RoadConditionManager
        manager = RoadConditionManager(
            degraded_edges, args.road_condition, degrade_start,
            v_low=args.degrade_vlow, v_high=args.degrade_vhigh,
            period_s=args.degrade_period, event_duration=args.event_duration
        )
'''
# inject manager into main just before "if args.paired:"
content = content.replace("    if args.paired:", main_manager + "    if args.paired:")

# Now we need to pass manager to run_paired_scenario_wrapper inside main()
# ex.submit(run_paired_scenario_wrapper, mode, alpha, beta, gamma, od_list, traffic_seed, tag, depart_start, depart_spacing, args.use_hysteresis, scale, teleport, debug_cfs=args.debug_cfs)
old_submit = "ex.submit(run_paired_scenario_wrapper, mode, alpha, beta, gamma, od_list, traffic_seed, tag, depart_start, depart_spacing, args.use_hysteresis, scale, teleport, debug_cfs=args.debug_cfs)"
new_submit = "ex.submit(run_paired_scenario_wrapper, mode, alpha, beta, gamma, od_list, traffic_seed, tag, depart_start, depart_spacing, args.use_hysteresis, scale, teleport, road_condition_manager=manager, debug_cfs=args.debug_cfs)"
content = content.replace(old_submit, new_submit)

with open(file_path, "w") as f:
    f.write(content)
print("Updated compare_routing.py phase 5 threading manager")
