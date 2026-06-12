import os
import re

file_path = "compare_routing.py"
with open(file_path, "r") as f:
    content = f.read()

# Replace run_paired_scenario signature to accept road_manager
old_sig = "def run_paired_scenario(arm_policy, alpha, beta, gamma, od_list, traffic_seed, tag,\n                        depart_start, depart_spacing, use_hysteresis,\n                        scale, teleport, debug_cfs=False):"
new_sig = "def run_paired_scenario(arm_policy, alpha, beta, gamma, od_list, traffic_seed, tag,\n                        depart_start, depart_spacing, use_hysteresis,\n                        scale, teleport, road_condition_manager=None, debug_cfs=False):"
content = content.replace(old_sig, new_sig)

# Replace sim = Simulation(...) instantiation in run_paired_scenario
old_sim = '''        sim = Simulation(
            net_file=NET_FILE,
            reroute_interval=REROUTE_INTERVAL,
            alpha=alpha, beta=beta, gamma=gamma,
            ego_type=EGO_TYPE,
            ego_routing=arm_policy if arm_policy != "ablation" else "ours",
            ego_od_list=od_list,
            debug_cfs=debug_cfs
        )'''
new_sim = '''        sim = Simulation(
            net_file=NET_FILE,
            reroute_interval=REROUTE_INTERVAL,
            alpha=alpha, beta=beta, gamma=gamma,
            ego_type=EGO_TYPE,
            ego_routing=arm_policy if arm_policy != "ablation" else "ours",
            ego_od_list=od_list,
            debug_cfs=debug_cfs,
            road_condition_manager=road_condition_manager
        )'''
content = content.replace(old_sim, new_sim)

with open(file_path, "w") as f:
    f.write(content)
print("Updated compare_routing.py phase 3 scenario signature")
