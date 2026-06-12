import os
import re

file_path = "compare_routing.py"
with open(file_path, "r") as f:
    content = f.read()

# Locate main OD generation
od_gen_idx = content.find("od_list = generate_od_pairs(NET_FILE, n=args.n, seed=args.od_seed)")

new_od_gen = '''
    depart_start = args.depart_start
    if args.degrade_start < 0:
        degrade_start = depart_start - 300
    else:
        degrade_start = args.degrade_start

    degraded_edges = []
    if args.road_condition != "none":
        if args.degraded_edges == "auto":
            degraded_edges = select_degraded_edges(NET_FILE, args.n_degraded, args.traffic_seed, args.scale, args.teleport, depart_start)
        elif args.degraded_edges:
            degraded_edges = [x.strip() for x in args.degraded_edges.split(",") if x.strip()]

    if args.verify_degradation:
        if not degraded_edges:
            sys.exit("Gate A Error: --verify-degradation requires degraded edges to be set.")
        print(f"\\n=======================================================")
        print(f" GATE A VERIFICATION: Running headless to measure fuel ")
        print(f"=======================================================")
        import traci
        from Simulation.simulate import RSUManager
        from Simulation.road_conditions import RoadConditionManager
        
        port = _free_port()
        sumo_cmd = [
            SUMO_BIN, "-c", CONFIG_FILE,
            "--seed", str(args.traffic_seed),
            "--scale", str(args.scale),
            "--no-step-log", "true",
            "--no-warnings", "true",
            "--time-to-teleport", str(args.teleport),
        ]
        traci.start(sumo_cmd, port=port)
        rsu_manager = RSUManager(record_interval=300)
        from RSU.edgecost import EdgeCostCalculator
        calc = EdgeCostCalculator(alpha=1.0, beta=1.0, gamma=1.0)
        rsu_manager.set_edge_cost_calc(calc)
        
        manager = RoadConditionManager(
            degraded_edges, args.road_condition, degrade_start,
            v_low=args.degrade_vlow, v_high=args.degrade_vhigh,
            period_s=args.degrade_period, event_duration=args.event_duration
        )
        manager.bind(traci, calc)
        
        fuel_before = {e: [] for e in degraded_edges}
        fuel_after = {e: [] for e in degraded_edges}
        time_before = {e: [] for e in degraded_edges}
        time_after = {e: [] for e in degraded_edges}
        
        end_time = depart_start + 1800  # Run for 30 minutes past ego depart
        
        while traci.simulation.getTime() < end_time:
            t = traci.simulation.getTime()
            traci.simulationStep()
            rsu_manager.step()
            manager.step(t)
            
            # Sample stats every 300s (at the end of an RSU window)
            if int(t) > 0 and int(t) % 300 == 0:
                for e in degraded_edges:
                    stats = rsu_manager.get_edge_stats(e)
                    if stats and stats.get("vehicle_count", 0) > 0:
                        f = stats.get("avg_fuel_mg", 0)
                        L = calc.edge_lengths.get(e, 100)
                        f_per_m = f / L if L > 0 else 0
                        tt = L / stats.get("avg_speed", 13.89)
                        
                        if t <= degrade_start:
                            fuel_before[e].append(f_per_m)
                            time_before[e].append(tt)
                        else:
                            fuel_after[e].append(f_per_m)
                            time_after[e].append(tt)
                            
        traci.close()
        
        print("\\nGate A Results (Fuel/meter and Traversal Time):")
        for e in degraded_edges:
            fb = sum(fuel_before[e])/len(fuel_before[e]) if fuel_before[e] else 0
            fa = sum(fuel_after[e])/len(fuel_after[e]) if fuel_after[e] else 0
            tb = sum(time_before[e])/len(time_before[e]) if time_before[e] else 0
            ta = sum(time_after[e])/len(time_after[e]) if time_after[e] else 0
            
            f_ratio = (fa/fb) if fb > 0 else 0
            t_ratio = (ta/tb) if tb > 0 else 0
            
            print(f"  Edge {e}:")
            print(f"    Fuel/m : Before={fb:.1f} mg/m, After={fa:.1f} mg/m -> Ratio = {f_ratio:.2f}x")
            print(f"    Time   : Before={tb:.1f} s, After={ta:.1f} s -> Ratio = {t_ratio:.2f}x")
            if f_ratio >= 1.3:
                print("    -> PASS (>= 30% fuel spike)")
            else:
                print("    -> FAIL (< 30% fuel spike)")
                
        sys.exit(0)

    if args.targeted_od and degraded_edges:
        od_list = select_targeted_od_pairs(NET_FILE, degraded_edges, n=args.n, seed=args.od_seed)
    else:
        od_list = generate_od_pairs(NET_FILE, n=args.n, seed=args.od_seed)
'''
content = content[:od_gen_idx] + new_od_gen + content[od_gen_idx + len("od_list = generate_od_pairs(NET_FILE, n=args.n, seed=args.od_seed)"):]

with open(file_path, "w") as f:
    f.write(content)
print("Updated compare_routing.py phase 4 main logic")
