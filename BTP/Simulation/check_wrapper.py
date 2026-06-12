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

# Fix concurrent futures submission in run_parallel (actually wait, let's just use regex to pass kwargs to submit)
# The submit call looks like: ex.submit(run_paired_scenario_wrapper, mode, alpha, beta, gamma, od_list, traffic_seed, tag, depart_start, depart_spacing, args.use_hysteresis, scale, teleport, debug_cfs=args.debug_cfs)
# Since I'm passing manager from main to run_parallel? Wait, run_parallel doesn't have args or manager.
# I need to create the manager in main, and pass it as an argument!
