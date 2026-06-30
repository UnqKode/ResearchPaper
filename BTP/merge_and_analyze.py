"""
merge_and_analyze.py
====================
Combines per-ego CSVs from two separate runs to produce a unified K=10 result.

Run 1 (verify_k10.txt)  : seeds 5, 7, 10 complete (both arms)
Run 2 (verify_k10_missing.txt): seeds 1,2,3,4,6,8,9 (both arms, re-run at parallelism=2)

Usage (from BTP/ directory):
    conda run -n ml python merge_and_analyze.py

The script auto-discovers the newest per-ego CSV files and filters by seed.
Override paths with:
    python merge_and_analyze.py --run1 <path1.csv> --run2 <path2.csv>
"""

import csv
import json
import glob
import os
import sys
import argparse
import time

try:
    from scipy import stats as scipy_stats
except ImportError:
    scipy_stats = None

# Seeds that came from each run
RUN1_SEEDS = {5, 7, 10}
RUN2_SEEDS = {1, 2, 3, 4, 6, 8, 9}
DEGRADED_EDGES = ["153391#0", "153391#1"]

def find_latest_ego_csv(exclude_path=None):
    pattern = os.path.join(os.path.dirname(__file__), "paired_per_ego_*_ours_vs_ablation.csv")
    files = sorted(glob.glob(pattern), key=os.path.getmtime, reverse=True)
    if exclude_path:
        files = [f for f in files if os.path.abspath(f) != os.path.abspath(exclude_path)]
    return files[0] if files else None


def load_ego_csv(path):
    rows = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            rows.append(r)
    return rows


def filter_seeds(rows, seeds):
    return [r for r in rows if int(r["seed"]) in seeds]


def analyze(rows, label="merged"):
    """Re-implements analyze_paired_results logic on a flat list of CSV rows."""

    def fv(r, key):
        v = r.get(key, "")
        try:
            return float(v) if v not in ("", "None", None) else None
        except ValueError:
            return None

    ours_by = {}
    base_by = {}
    for r in rows:
        seed = int(r["seed"])
        trip = int(r["trip"])
        arm  = r["arm"]
        if arm == "ours":
            ours_by[(seed, trip)] = r
        elif arm == "ablation":
            base_by[(seed, trip)] = r

    keys = sorted(set(ours_by) | set(base_by))
    seeds_seen = sorted(set(k[0] for k in keys))

    seed_records = []
    print(f"\n{'='*60}")
    print(f" MERGED RESULT  ({label})")
    print(f"{'='*60}")

    for s in seeds_seen:
        seed_keys = [k for k in keys if k[0] == s]

        ours_arrive   = [k for k in seed_keys if k in ours_by and ours_by[k].get("arrived") == "True"]
        base_arrive   = [k for k in seed_keys if k in base_by and base_by[k].get("arrived") == "True"]
        ours_noarrive = [k for k in seed_keys if k in ours_by and ours_by[k].get("arrived") != "True"]
        base_noarrive = [k for k in seed_keys if k in base_by and base_by[k].get("arrived") != "True"]

        print(f"\n[ARRIVAL_COUNTS] seed={s}: "
              f"ours={len(ours_arrive)} arrived / {len(ours_noarrive)} did-not-arrive; "
              f"ablation={len(base_arrive)} arrived / {len(base_noarrive)} did-not-arrive")
        if ours_noarrive:
            print(f"  ours non-arrivals: trips {sorted(k[1] for k in ours_noarrive)}")
        if base_noarrive:
            print(f"  ablation non-arrivals: trips {sorted(k[1] for k in base_noarrive)}")

        paired = [
            k for k in seed_keys
            if k in ours_by and k in base_by
            and ours_by[k].get("arrived") == "True"
            and base_by[k].get("arrived") == "True"
            and fv(ours_by[k], "fuel_abs") is not None
            and fv(base_by[k], "fuel_abs") is not None
        ]

        if not paired:
            print(f"  => seed={s}: no paired trips, skipping")
            continue

        o_fuel = [fv(ours_by[k], "fuel_abs") for k in paired]
        b_fuel = [fv(base_by[k], "fuel_abs") for k in paired]
        o_dur  = [fv(ours_by[k], "duration")  for k in paired]
        b_dur  = [fv(base_by[k], "duration")  for k in paired]
        o_co2  = [fv(ours_by[k], "CO2_abs")   for k in paired]
        b_co2  = [fv(base_by[k], "CO2_abs")   for k in paired]

        fuel_sav = [100*(b-o)/b for o,b in zip(o_fuel,b_fuel) if b > 0]
        dur_sav  = [100*(b-o)/b for o,b in zip(o_dur, b_dur)  if b > 0]
        co2_sav  = [100*(b-o)/b for o,b in zip(o_co2, b_co2)  if b > 0]

        mean_fuel = sum(fuel_sav)/len(fuel_sav)
        mean_dur  = sum(dur_sav)/len(dur_sav)

        print(f"  => seed={s}: n_paired={len(paired)}, "
              f"fuel_sav={mean_fuel:+.2f}%, dur_sav={mean_dur:+.2f}%")

        seed_records.append({
            "seed": s,
            "n_paired": len(paired),
            "ours_fuel": sum(o_fuel)/len(o_fuel),
            "base_fuel": sum(b_fuel)/len(b_fuel),
            "ours_dur":  sum(o_dur)/len(o_dur),
            "base_dur":  sum(b_dur)/len(b_dur),
            "ours_co2":  sum(o_co2)/len(o_co2) if o_co2 else None,
            "base_co2":  sum(b_co2)/len(b_co2) if b_co2 else None,
            "fuel_saving_pct": mean_fuel,
            "dur_saving_pct":  mean_dur,
        })

    K = len(seed_records)
    print(f"\n{'='*60}")
    print(f" CROSS-SEED SUMMARY  K={K} seeds")
    print(f"{'='*60}")

    if K == 0:
        print("No complete paired seeds found. Exiting.")
        return None

    fuel_sav = [sr["fuel_saving_pct"] for sr in seed_records]
    dur_sav  = [sr["dur_saving_pct"]  for sr in seed_records]

    mean_f = sum(fuel_sav)/K
    mean_d = sum(dur_sav)/K
    print(f" Mean fuel saving : {mean_f:+.2f}%")
    print(f" Mean time saving : {mean_d:+.2f}%")

    summary = {"K_seeds": K, "fuel_sav_mean": mean_f, "dur_sav_mean": mean_d}

    if scipy_stats and K > 1:
        o_f = [sr["ours_fuel"] for sr in seed_records]
        b_f = [sr["base_fuel"] for sr in seed_records]
        o_d = [sr["ours_dur"]  for sr in seed_records]
        b_d = [sr["base_dur"]  for sr in seed_records]

        _, p_f = scipy_stats.ttest_rel(o_f, b_f)
        diffs_f = [o-b for o,b in zip(o_f, b_f)]
        std_f = (sum((x - sum(diffs_f)/K)**2 for x in diffs_f)/(K-1))**0.5
        dz_f = (sum(diffs_f)/K) / std_f if std_f > 0 else 0
        se_f = std_f / K**0.5
        t_crit = scipy_stats.t.ppf(0.975, df=K-1)
        mean_pct = sum(fuel_sav)/K
        half_w = t_crit * (sum((x-mean_pct)**2 for x in fuel_sav)/(K-1))**0.5 / K**0.5
        ci_lo, ci_hi = mean_pct - half_w, mean_pct + half_w

        try:
            _, w_p = scipy_stats.wilcoxon(diffs_f)
        except Exception:
            w_p = None

        print(f"\n Fuel paired t-test : p={p_f:.4e}")
        print(f" Fuel Wilcoxon      : p={w_p:.4f}" if w_p is not None else " Fuel Wilcoxon: N/A")
        print(f" Cohen d_z          : {dz_f:.3f}")
        print(f" 95% CI (fuel %)    : [{ci_lo:+.2f}%, {ci_hi:+.2f}%]")

        summary["fuel_stats"] = {
            "t_test_p": p_f, "wilcoxon_p": w_p,
            "cohen_dz": dz_f,
            "CI_95_pct": [ci_lo, ci_hi],
        }

    # Gate C: avoidance of degraded edges
    ours_gate, base_gate = [], []
    for r in rows:
        if r.get("arrived") != "True":
            continue
        driven = r.get("driven_edges", "")
        if isinstance(driven, str):
            # CSV stores driven_edges as a string representation; skip if unavailable
            continue
        on_deg = any(e in DEGRADED_EDGES for e in driven)
        if r["arm"] == "ours":
            ours_gate.append(not on_deg)
        elif r["arm"] == "ablation":
            base_gate.append(not on_deg)

    if ours_gate or base_gate:
        or_ = 100*sum(ours_gate)/len(ours_gate) if ours_gate else 0
        br_ = 100*sum(base_gate)/len(base_gate) if base_gate else 0
        print(f"\n Gate C avoidance: ours={or_:.1f}%  ablation={br_:.1f}%")
        summary["gate_c"] = {"avoidance_rate_ours_pct": or_, "avoidance_rate_ablation_pct": br_}

    # Per-seed table
    print(f"\n{'seed':>6} {'n_paired':>8} {'fuel_sav%':>10} {'dur_sav%':>10}")
    print(f"  {'-'*40}")
    for sr in seed_records:
        print(f"  {sr['seed']:>4}   {sr['n_paired']:>8}   {sr['fuel_saving_pct']:>+9.2f}%  {sr['dur_saving_pct']:>+9.2f}%")

    stamp = time.strftime("%Y%m%d_%H%M%S")
    out_path = os.path.join(os.path.dirname(__file__), f"merged_summary_{stamp}.json")
    with open(out_path, "w") as f:
        json.dump(summary, f, indent=2)
    print(f"\n Saved: {out_path}")
    return summary


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run1", default=None,
                    help="per-ego CSV from run 1 (seeds 5,7,10). Auto-detected if omitted.")
    ap.add_argument("--run2", default=None,
                    help="per-ego CSV from run 2 (seeds 1,2,3,4,6,8,9). Auto-detected if omitted.")
    args = ap.parse_args()

    # Auto-detect CSVs
    all_csvs = sorted(
        glob.glob(os.path.join(os.path.dirname(__file__), "paired_per_ego_*_ours_vs_ablation.csv")),
        key=os.path.getmtime
    )

    if args.run1:
        run1_path = args.run1
    elif all_csvs:
        run1_path = all_csvs[0]   # oldest = first run
        print(f"Auto-detected run1 CSV: {os.path.basename(run1_path)}")
    else:
        sys.exit("No per-ego CSV found. Pass --run1 explicitly.")

    if args.run2:
        run2_path = args.run2
    elif len(all_csvs) >= 2:
        run2_path = all_csvs[-1]  # newest = second run
        print(f"Auto-detected run2 CSV: {os.path.basename(run2_path)}")
    else:
        print("Run2 CSV not yet available — showing run1 results only.")
        run2_path = None

    rows1 = load_ego_csv(run1_path)
    r1 = filter_seeds(rows1, RUN1_SEEDS)
    print(f"Run1 rows loaded: {len(rows1)} total, {len(r1)} for seeds {sorted(RUN1_SEEDS)}")

    r2 = []
    if run2_path:
        rows2 = load_ego_csv(run2_path)
        r2 = filter_seeds(rows2, RUN2_SEEDS)
        print(f"Run2 rows loaded: {len(rows2)} total, {len(r2)} for seeds {sorted(RUN2_SEEDS)}")

        # Sanity check: no seed overlap
        seeds1 = set(int(r["seed"]) for r in r1)
        seeds2 = set(int(r["seed"]) for r in r2)
        overlap = seeds1 & seeds2
        if overlap:
            print(f"WARNING: seed overlap between runs: {overlap}. Run1 data takes priority.")
            r2 = [r for r in r2 if int(r["seed"]) not in overlap]

    merged = r1 + r2
    seeds_present = sorted(set(int(r["seed"]) for r in merged))
    print(f"\nMerged dataset: {len(merged)} rows, seeds present: {seeds_present}")

    analyze(merged, label=f"run1(seeds={sorted(RUN1_SEEDS)}) + run2(seeds={sorted(seeds_present - RUN1_SEEDS)})")


if __name__ == "__main__":
    main()
