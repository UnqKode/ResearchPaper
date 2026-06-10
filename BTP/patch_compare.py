import os
import sys

filepath = r"C:\Users\Manas Yadav\OneDrive\Documents\RESEARCH\MoST\MoSTScenario\BTP\Simulation\compare_routing.py"

with open(filepath, "r", encoding="utf-8") as f:
    content = f.read()

# 1. Replace parse_tripinfo
old_parse = '''def parse_tripinfo(path):
    """
    Parse SUMO's tripinfo. Returns {veh_id: {"duration": s, "fuel_mg": mg}}.
    fuel_abs in tripinfo emissions is in mg (SUMO reports mg for the HBEFA fuel
    output). If the emissions block is absent (vType has no fuel model), fuel is
    left as None and we fall back to the live TraCI integration.
    """
    out = {}
    if not os.path.exists(path):
        print(f"  (no tripinfo at {path}; using live metrics only)")
        return out
    try:
        tree = ET.parse(path)
    except ET.ParseError:
        return out
    for ti in tree.getroot().findall("tripinfo"):
        vid = ti.get("id")
        dur = ti.get("duration")
        rec = {"duration": float(dur) if dur is not None else None,
               "fuel_mg": None}
        em = ti.find("emissions")
        if em is not None and em.get("fuel_abs") is not None:
            rec["fuel_mg"] = float(em.get("fuel_abs"))
        out[vid] = rec
    return out'''

new_parse = '''def parse_tripinfo(path):
    out = {}
    if not os.path.exists(path):
        return out
    try:
        tree = ET.parse(path)
    except ET.ParseError:
        return out
    for ti in tree.getroot().findall("tripinfo"):
        vid = ti.get("id")
        rec = {
            "duration": float(ti.get("duration")) if ti.get("duration") is not None else None,
            "routeLength": float(ti.get("routeLength")) if ti.get("routeLength") is not None else None,
            "waitingTime": float(ti.get("waitingTime")) if ti.get("waitingTime") is not None else None,
            "waitingCount": int(ti.get("waitingCount")) if ti.get("waitingCount") is not None else None,
            "timeLoss": float(ti.get("timeLoss")) if ti.get("timeLoss") is not None else None,
            "fuel_mg": None,
            "CO2_mg": None
        }
        em = ti.find("emissions")
        if em is not None:
            if em.get("fuel_abs") is not None:
                rec["fuel_mg"] = float(em.get("fuel_abs"))
            if em.get("CO2_abs") is not None:
                rec["CO2_mg"] = float(em.get("CO2_abs"))
        out[vid] = rec
    return out'''

content = content.replace(old_parse, new_parse)

# 2. Add run_paired_scenario and run_paired_capture and stats logic before main()
injection = '''
# ===========================================================================
# PAIRED SAME-SEED FUEL CAMPAIGN
# ===========================================================================
def run_paired_scenario(arm_policy, alpha, beta, gamma, od_list, traffic_seed, tag,
                        depart_start, depart_spacing, use_hysteresis,
                        scale, teleport):
    out_prefix = f"{tag}."
    tripinfo_path = f"{out_prefix}tripinfo.xml"
    progress_path = f"progress_{tag}.csv"

    try: os.remove(tripinfo_path)
    except OSError: pass

    # Compute end time
    n = len(od_list)
    horizon_s = depart_start + int(n * depart_spacing) + int(PER_TRIP_TIMEOUT * 0.25) + 3600
    port = _free_port()
    sumo_cmd = [
        SUMO_BIN, "-c", CONFIG_FILE,
        "--seed", str(traffic_seed),
        "--scale", str(scale),
        "--no-step-log", "true",
        "--no-warnings", "true",
        "--time-to-teleport", str(teleport),
        "--end", str(int(horizon_s)),
        "--output-prefix", out_prefix,
        "--tripinfo-output", "tripinfo.xml",
        "--log", "sim.log",
        "--error-log", "errors.log",
        # Background routing ON for paired mode
        "--device.rerouting.probability", "1",
        "--device.rerouting.period", str(REROUTE_INTERVAL),
        "--route-steps", "0"
    ]
    started = False
    live_results = []
    try:
        traci.start(sumo_cmd, port=port)
        started = True
        sim = Simulation(
            net_file=NET_FILE,
            reroute_interval=REROUTE_INTERVAL,
            alpha=alpha, beta=beta, gamma=gamma,
            ego_type=EGO_TYPE,
            ego_routing=arm_policy if arm_policy != "ablation" else "ours",
            ego_od_list=od_list
        )
        live_results = sim.run_fixed_departure_campaign(
            od_list=od_list,
            depart_start=depart_start,
            depart_spacing=depart_spacing,
            per_trip_timeout=PER_TRIP_TIMEOUT,
            ego_policy=arm_policy,
            reroute_interval=REROUTE_INTERVAL,
            use_hysteresis=use_hysteresis,
            progress_log_path=progress_path
        )
    except traci.FatalTraCIError as e:
        print(f"TraCI error in paired scenario '{tag}': {e}")
    finally:
        if started:
            try: traci.close()
            except: pass
            sys.stdout.flush()
    
    deadline = time.time() + TRIPINFO_FLUSH_WAIT_S
    while time.time() < deadline:
        if os.path.exists(tripinfo_path) and os.path.getsize(tripinfo_path) > 0:
            break
        time.sleep(0.25)
    
    tripinfo = parse_tripinfo(tripinfo_path)
    # Join tripinfo into live_results by ego id
    for rec in live_results:
        vid = f"ego_{rec['trip']}"
        rec["seed"] = traffic_seed
        rec["arm"] = arm_policy
        ti = tripinfo.get(vid, {})
        rec["duration"] = ti.get("duration")
        rec["routeLength"] = ti.get("routeLength")
        rec["waitingTime"] = ti.get("waitingTime")
        rec["waitingCount"] = ti.get("waitingCount")
        rec["timeLoss"] = ti.get("timeLoss")
        rec["fuel_abs"] = ti.get("fuel_mg")
        rec["CO2_abs"] = ti.get("CO2_mg")
        if rec["arrived"] and ti.get("fuel_mg") is not None and ti.get("routeLength", 0) > 0:
            rec["fuel_per_km"] = ti["fuel_mg"] / (ti["routeLength"] / 1000.0)
        else:
            rec["fuel_per_km"] = None
    return live_results

def _run_paired_capture(arm_policy, alpha, beta, gamma, od_list, traffic_seed, tag,
                        depart_start, depart_spacing, use_hysteresis, scale, teleport):
    t0 = time.time()
    buf = io.StringIO()
    error = None
    res = []
    try:
        with redirect_stdout(buf):
            res = run_paired_scenario(arm_policy, alpha, beta, gamma, od_list, traffic_seed, tag,
                                      depart_start, depart_spacing, use_hysteresis, scale, teleport)
    except Exception as e:
        error = repr(e)
    return {
        "tag": tag, "log": buf.getvalue(), "error": error,
        "results": res, "wall_s": time.time() - t0
    }

def analyze_paired_results(ours_lists, base_lists, base_name, run_meta):
    import json
    try: from scipy import stats
    except ImportError: stats = None
    
    stamp = time.strftime("%Y%m%d_%H%M%S")
    f_ego = f"paired_per_ego_{stamp}_ours_vs_{base_name}.csv"
    f_seed = f"paired_per_seed_{stamp}_ours_vs_{base_name}.csv"
    f_sum = f"paired_summary_{stamp}_ours_vs_{base_name}.json"
    
    ours_flat = [r for sub in ours_lists for r in sub]
    base_flat = [r for sub in base_lists for r in sub]
    
    ours_by = {(r["seed"], r["trip"]): r for r in ours_flat}
    base_by = {(r["seed"], r["trip"]): r for r in base_flat}
    keys = sorted(set(ours_by) | set(base_by))
    
    # 1. Per-Ego CSV
    with open(f_ego, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["seed", "trip", "arm", "origin", "dest", "arrived", "fuel_abs",
                    "fuel_per_km", "duration", "CO2_abs", "waitingTime", "stops",
                    "timeLoss", "routeLength", "reroutes", "bg_vcount_at_depart",
                    "bg_possum_at_depart"])
        for flat in [base_flat, ours_flat]:
            for r in flat:
                w.writerow([
                    r["seed"], r["trip"], r["arm"], r["origin"], r["dest"], r["arrived"],
                    r.get("fuel_abs"), r.get("fuel_per_km"), r.get("duration"),
                    r.get("CO2_abs"), r.get("waitingTime"), r.get("waitingCount"),
                    r.get("timeLoss"), r.get("routeLength"), r.get("reroutes"),
                    r.get("bg_vcount_at_depart"), r.get("bg_possum_at_depart")
                ])
                
    # Level 1: Seed Aggregation
    seeds = sorted(list(set(k[0] for k in keys)))
    seed_records = []
    
    for s in seeds:
        seed_keys = [k for k in keys if k[0] == s]
        paired = [k for k in seed_keys if k in ours_by and k in base_by and
                  ours_by[k]["arrived"] and base_by[k]["arrived"] and
                  ours_by[k].get("fuel_abs") is not None and base_by[k].get("fuel_abs") is not None]
        
        if not paired: continue
        
        o_fuel = [ours_by[k]["fuel_abs"] for k in paired]
        b_fuel = [base_by[k]["fuel_abs"] for k in paired]
        o_dur = [ours_by[k]["duration"] for k in paired]
        b_dur = [base_by[k]["duration"] for k in paired]
        o_co2 = [ours_by[k]["CO2_abs"] for k in paired]
        b_co2 = [base_by[k]["CO2_abs"] for k in paired]
        
        fuel_sav_pct = [100 * (b - o) / b if b > 0 else 0 for o, b in zip(o_fuel, b_fuel)]
        dur_sav_pct = [100 * (b - o) / b if b > 0 else 0 for o, b in zip(o_dur, b_dur)]
        co2_sav_pct = [100 * (b - o) / b if b > 0 else 0 for o, b in zip(o_co2, b_co2)]
        
        seed_records.append({
            "seed": s,
            "n_paired": len(paired),
            "ours_fuel": sum(o_fuel)/len(o_fuel), "base_fuel": sum(b_fuel)/len(b_fuel),
            "ours_dur": sum(o_dur)/len(o_dur), "base_dur": sum(b_dur)/len(b_dur),
            "ours_co2": sum(o_co2)/len(o_co2), "base_co2": sum(b_co2)/len(b_co2),
            "fuel_saving_pct": sum(fuel_sav_pct)/len(fuel_sav_pct),
            "dur_saving_pct": sum(dur_sav_pct)/len(dur_sav_pct),
            "co2_saving_pct": sum(co2_sav_pct)/len(co2_sav_pct),
        })
        
    with open(f_seed, "w", newline="") as f:
        w = csv.writer(f)
        if seed_records:
            w.writerow(list(seed_records[0].keys()))
            for sr in seed_records: w.writerow(list(sr.values()))

    # Level 2: Cross-seed stats
    K = len(seed_records)
    summary = {"run_meta": run_meta, "K_seeds": K}
    print(f"\\n=======================================================")
    print(f" PAIRED FUEL RESULTS: OURS vs {base_name.upper()}")
    print(f"=======================================================")
    print(f" N (seeds) = {K}")
    if K > 0:
        fuel_sav = [sr["fuel_saving_pct"] for sr in seed_records]
        dur_sav = [sr["dur_saving_pct"] for sr in seed_records]
        co2_sav = [sr["co2_saving_pct"] for sr in seed_records]
        
        summary["fuel_sav_mean"] = sum(fuel_sav)/K
        summary["dur_sav_mean"] = sum(dur_sav)/K
        summary["co2_sav_mean"] = sum(co2_sav)/K
        
        print(f" Mean Fuel Saving: {summary['fuel_sav_mean']:+.2f}%")
        print(f" Mean Time Saving: {summary['dur_sav_mean']:+.2f}%")
        
        if stats and K > 1:
            try:
                # Level 2 t-test on seed means
                o_f = [sr["ours_fuel"] for sr in seed_records]
                b_f = [sr["base_fuel"] for sr in seed_records]
                t_f, p_f = stats.ttest_rel(o_f, b_f)
                try: _, w_p_f = stats.wilcoxon([o-b for o,b in zip(o_f, b_f)])
                except: w_p_f = None
                
                # Cohen's d_z = mean(diff) / std(diff)
                diffs = [b - o for b, o in zip(b_f, o_f)]
                mean_d = sum(diffs)/K
                std_d = (sum((d - mean_d)**2 / (K - 1)) for d in diffs)
                std_d = sum((d - mean_d)**2 for d in diffs) / (K - 1)
                std_d = std_d ** 0.5
                dz = mean_d / std_d if std_d > 0 else 0
                
                # 95% CI
                ci_err = stats.t.ppf(0.975, K-1) * (std_d / (K**0.5))
                ci = (mean_d - ci_err, mean_d + ci_err)
                
                # convert CI from absolute mg to roughly %
                mean_b = sum(b_f)/K
                ci_pct = (ci[0]/mean_b*100, ci[1]/mean_b*100)
                
                summary["fuel_stats"] = {
                    "t_test_p": p_f, "wilcoxon_p": w_p_f, "cohen_dz": dz, "CI_95_pct": ci_pct
                }
                print(f" Fuel Stats: paired t-test p={p_f:.4f}, Wilcoxon p={w_p_f}")
                print(f" Fuel 95% CI of savings: [{ci_pct[0]:.2f}%, {ci_pct[1]:.2f}%]")
            except Exception as e: print(f" Stats failed: {e}")
            
    with open(f_sum, "w") as f: json.dump(summary, f, indent=2)

'''

content = content.replace("def main():", injection + "\ndef main():")

# 3. Add args to main()
old_args = '''    ap.add_argument("--teleport", type=int, default=DEFAULT_TELEPORT,
                    help="seconds a vehicle may be stuck before SUMO teleports it "
                         "(HP-2). -1 = never (honest timing but can deadlock at high "
                         "--scale); 300 is a safe finite default")
    args = ap.parse_args()'''

new_args = '''    ap.add_argument("--teleport", type=int, default=DEFAULT_TELEPORT,
                    help="seconds a vehicle may be stuck before SUMO teleports it "
                         "(HP-2). -1 = never (honest timing but can deadlock at high "
                         "--scale); 300 is a safe finite default")
    ap.add_argument("--paired", action="store_true", help="enable same-seed paired-run fuel experiment")
    ap.add_argument("--baseline", choices=["sumo", "ablation", "both"], default="both", help="baseline to compare ours against")
    ap.add_argument("--depart-start", type=float, default=21600, help="sim time of first ego departure")
    ap.add_argument("--depart-spacing", type=float, default=120, help="steps between ego departures")
    ap.add_argument("--reroute-interval", type=int, default=REROUTE_INTERVAL, help="reroute cadence")
    ap.add_argument("--use-hysteresis", action="store_true", help="re-enable ours hysteresis")
    args = ap.parse_args()'''

content = content.replace(old_args, new_args)

# 4. Inject paired mode at the top of main logic
old_main_body = '''    if args.legacy:'''
new_main_body = '''    if args.paired:
        print("\\n[Paired Mode] Running fixed-departure continuous fuel campaign.")
        seeds = [int(x.strip()) for x in args.seeds.split(",")]
        run_meta = {
            "n": args.n, "od_seed": args.od_seed, "seeds": seeds,
            "scale": args.scale, "teleport": args.teleport,
            "alpha": ALPHA, "beta": BETA, "gamma": GAMMA,
            "depart_start": args.depart_start, "depart_spacing": args.depart_spacing,
            "reroute_interval": args.reroute_interval, "use_hysteresis": args.use_hysteresis,
            "baseline": args.baseline
        }
        
        arms_to_run = ["ours"]
        if args.baseline in ["sumo", "both"]: arms_to_run.append("sumo")
        if args.baseline in ["ablation", "both"]: arms_to_run.append("ablation")
        
        all_res = {arm: [] for arm in arms_to_run}
        
        for seed in seeds:
            print(f"\\n--- SEED {seed} ---")
            ctx = multiprocessing.get_context("spawn")
            ex = concurrent.futures.ProcessPoolExecutor(max_workers=len(arms_to_run), mp_context=ctx)
            
            futs = {}
            for arm in arms_to_run:
                # for ablation, alpha/beta/gamma are 0
                a, b, g = (0.0, 0.0, 0.0) if arm == "ablation" else (ALPHA, BETA, GAMMA)
                fut = ex.submit(_run_paired_capture, arm, a, b, g, od_list, seed, f"{arm}_{seed}",
                                args.depart_start, args.depart_spacing, args.use_hysteresis,
                                args.scale, args.teleport)
                futs[fut] = arm
                
            for fut in concurrent.futures.as_completed(futs, timeout=WORKER_TIMEOUT_S):
                arm = futs[fut]
                out = fut.result()
                print(f"\\n# scenario '{out['tag']}' finished in {out['wall_s']:.1f}s")
                if out["error"]: print(f"[ERROR] {out['error']}")
                all_res[arm].append(out["results"])
            ex.shutdown(wait=False, cancel_futures=True)
            
        if "sumo" in all_res:
            analyze_paired_results(all_res["ours"], all_res["sumo"], "sumo", run_meta)
        if "ablation" in all_res:
            analyze_paired_results(all_res["ours"], all_res["ablation"], "ablation", run_meta)
            
        return

    if args.legacy:'''

content = content.replace(old_main_body, new_main_body)

with open(filepath, "w", encoding="utf-8") as f:
    f.write(content)

print("Patch applied.")
