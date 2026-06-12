import os
import re

file_path = "compare_routing.py"
with open(file_path, "r") as f:
    content = f.read()

# Add logic to analyze_paired_results
target_fn = "def analyze_paired_results(armA, armB, nameB, run_meta):"
new_fn = '''def analyze_paired_results(armA, armB, nameB, run_meta):
    print(f"\\n=======================================================")
    print(f" PAIRED METRICS (ours vs {nameB}) ")
    print(f"=======================================================")
    
    # Calculate Avoidance (Gate C) if degraded edges exist
    degraded_edges = run_meta.get("degraded_edges", [])
    avoidance_A = 0
    avoidance_B = 0
    total_A = 0
    total_B = 0
    
    if degraded_edges:
        for tA in armA:
            if tA.get("arrived"):
                total_A += 1
                driven = tA.get("driven_edges", [])
                if not any(de in driven for de in degraded_edges):
                    avoidance_A += 1
        for tB in armB:
            if tB.get("arrived"):
                total_B += 1
                driven = tB.get("driven_edges", [])
                if not any(de in driven for de in degraded_edges):
                    avoidance_B += 1
                    
        rate_A = (avoidance_A / total_A * 100) if total_A > 0 else 0
        rate_B = (avoidance_B / total_B * 100) if total_B > 0 else 0
        print(f"\\nGate C Avoidance Rate (fraction avoiding ALL degraded edges):")
        print(f"  ours : {avoidance_A}/{total_A} ({rate_A:.1f}%)")
        print(f"  {nameB} : {avoidance_B}/{total_B} ({rate_B:.1f}%)")
'''

content = content.replace("def analyze_paired_results(armA, armB, nameB, run_meta):\n", new_fn)

# In main(), I need to add degraded_edges to run_meta
old_run_meta = '''        run_meta = {
            "n": args.n, "od_seed": args.od_seed, "seeds": seeds,
            "scale": args.scale, "teleport": args.teleport,
            "alpha": ALPHA, "beta": BETA, "gamma": GAMMA,
            "reroute_interval": REROUTE_INTERVAL, "warmup_steps": WARMUP_STEPS,
            "per_trip_timeout": PER_TRIP_TIMEOUT, "ego_type": EGO_TYPE,
            "od_pairs_built": len(od_list),
        }'''
new_run_meta = '''        run_meta = {
            "n": args.n, "od_seed": args.od_seed, "seeds": seeds,
            "scale": args.scale, "teleport": args.teleport,
            "alpha": ALPHA, "beta": BETA, "gamma": GAMMA,
            "reroute_interval": REROUTE_INTERVAL, "warmup_steps": WARMUP_STEPS,
            "per_trip_timeout": PER_TRIP_TIMEOUT, "ego_type": EGO_TYPE,
            "od_pairs_built": len(od_list),
            "degraded_edges": degraded_edges,
            "road_condition": args.road_condition,
        }'''
content = content.replace(old_run_meta, new_run_meta)

with open(file_path, "w") as f:
    f.write(content)
print("Updated compare_routing.py phase 6 gate C avoidance")
