"""
Step 3B: Independent recompute of headline numbers from paired_per_ego CSV.
Run from BTP/ after the big run completes:
  conda run -n ml python audit_recompute.py

Reads paired_per_ego_*.csv (the raw per-ego rows written by analyze_paired_results)
and recomputes:
  - per-seed mean fuel% and time% (ours vs ablation, paired arrivals only)
  - across-seed mean, 95% CI, paired t-test, Cohen's d_z
WITHOUT using the summary JSON — this is an independent aggregation check.
"""
import csv
import glob
import json
import math
import os
import sys

# ---------------------------------------------------------------------------
# Find artifacts
# ---------------------------------------------------------------------------
ego_files = sorted(glob.glob("paired_per_ego_*.csv"))
seed_files = sorted(glob.glob("paired_per_seed_*.csv"))
sum_files  = sorted(glob.glob("paired_summary_*.json"))

if not ego_files:
    sys.exit("No paired_per_ego_*.csv found. Run the big verification first.")

ego_csv = ego_files[-1]   # most recent
sum_json = sum_files[-1] if sum_files else None

print(f"Reading per-ego CSV: {ego_csv}")
print(f"Reference JSON:      {sum_json}")
print()

# ---------------------------------------------------------------------------
# Load per-ego rows
# ---------------------------------------------------------------------------
rows = []
with open(ego_csv, newline="") as f:
    reader = csv.DictReader(f)
    for row in reader:
        rows.append(row)

print(f"Total rows in CSV: {len(rows)}")

# Group by (seed, trip, arm)
ours_by = {}   # (seed, trip) -> row
base_by = {}   # (seed, trip) -> row

for row in rows:
    seed = int(row["seed"])
    trip = int(row["trip"])
    arm  = row["arm"]
    if arm == "ours":
        ours_by[(seed, trip)] = row
    elif arm == "ablation":
        base_by[(seed, trip)] = row

# ---------------------------------------------------------------------------
# Per-seed aggregation (replicating analyze_paired_results logic)
# ---------------------------------------------------------------------------
seeds = sorted(set(s for s, _ in set(ours_by) | set(base_by)))
print(f"Seeds found: {seeds}")
print()

seed_records = []
for s in seeds:
    all_trips = set(t for ss, t in ours_by if ss == s) | set(t for ss, t in base_by if ss == s)

    paired_keys = []
    for t in sorted(all_trips):
        o_row = ours_by.get((s, t))
        b_row = base_by.get((s, t))
        if (o_row and b_row and
                o_row.get("arrived", "").lower() == "true" and
                b_row.get("arrived", "").lower() == "true" and
                o_row.get("fuel_abs") not in ("", "None", None) and
                b_row.get("fuel_abs") not in ("", "None", None)):
            paired_keys.append(t)

    if not paired_keys:
        print(f"  Seed {s}: no paired arrivals")
        continue

    o_fuel = [float(ours_by[(s, t)]["fuel_abs"]) for t in paired_keys]
    b_fuel = [float(base_by[(s, t)]["fuel_abs"]) for t in paired_keys]
    o_dur  = [float(ours_by[(s, t)]["duration"])  for t in paired_keys]
    b_dur  = [float(base_by[(s, t)]["duration"])  for t in paired_keys]

    fuel_sav_pct = [100 * (b - o) / b for o, b in zip(o_fuel, b_fuel) if b > 0]
    dur_sav_pct  = [100 * (b - o) / b for o, b in zip(o_dur,  b_dur)  if b > 0]

    mean_fuel = sum(fuel_sav_pct) / len(fuel_sav_pct)
    mean_dur  = sum(dur_sav_pct)  / len(dur_sav_pct)

    ours_noarrive = sum(1 for t in all_trips if ours_by.get((s,t)) and ours_by[(s,t)].get("arrived","").lower()!="true")
    base_noarrive = sum(1 for t in all_trips if base_by.get((s,t)) and base_by[(s,t)].get("arrived","").lower()!="true")

    print(f"  Seed {s}: n_paired={len(paired_keys)}, "
          f"ours_noarrive={ours_noarrive}, ablation_noarrive={base_noarrive}")
    print(f"           fuel_saving={mean_fuel:+.2f}%, time_saving={mean_dur:+.2f}%")

    seed_records.append({
        "seed": s,
        "n_paired": len(paired_keys),
        "ours_fuel_mean": sum(o_fuel)/len(o_fuel),
        "base_fuel_mean": sum(b_fuel)/len(b_fuel),
        "ours_dur_mean":  sum(o_dur) /len(o_dur),
        "base_dur_mean":  sum(b_dur) /len(b_dur),
        "fuel_saving_pct": mean_fuel,
        "dur_saving_pct":  mean_dur,
    })

# ---------------------------------------------------------------------------
# Across-seed stats
# ---------------------------------------------------------------------------
K = len(seed_records)
print(f"\n{'='*60}")
print(f"ACROSS-SEED RECOMPUTE  (K={K} seeds)")
print(f"{'='*60}")

if K == 0:
    sys.exit("No seed records — cannot aggregate.")

fuel_sav_k = [r["fuel_saving_pct"] for r in seed_records]
dur_sav_k  = [r["dur_saving_pct"]  for r in seed_records]

def stats_of(vals, label):
    n = len(vals)
    mean_v = sum(vals) / n
    var_v  = sum((x - mean_v)**2 for x in vals) / (n - 1) if n > 1 else 0.0
    std_v  = math.sqrt(var_v)
    # 95% CI via t-distribution (two-tailed, df=n-1)
    try:
        from scipy import stats as sp
        t_crit = sp.t.ppf(0.975, n - 1)
        se = std_v / math.sqrt(n)
        ci_lo, ci_hi = mean_v - t_crit * se, mean_v + t_crit * se
        # Cohen's d_z = mean_diff / std_diff  (using ours-base differences directly)
        # But we have per-seed means, so use those directly.
        # The "difference" population is fuel_saving (= base - ours in absolute mg / base * 100).
        # For d_z we want mean(diff)/std(diff) across seeds.
        # diff = ours_fuel - base_fuel  (negative = ours used less)
        o_means = [r["ours_fuel_mean"] for r in seed_records]
        b_means = [r["base_fuel_mean"] for r in seed_records]
        diffs = [o - b for o, b in zip(o_means, b_means)]
        mean_d = sum(diffs) / n
        std_d  = math.sqrt(sum((d - mean_d)**2 for d in diffs) / (n - 1)) if n > 1 else 0.0
        dz     = mean_d / std_d if std_d > 0 else float("nan")
        _, p   = sp.ttest_rel(o_means, b_means)
        print(f"\n{label}")
        print(f"  Mean saving:  {mean_v:+.2f}%  (negative = ours used less)")
        print(f"  Std:          {std_v:.2f}%")
        print(f"  95% CI:       [{ci_lo:+.2f}%, {ci_hi:+.2f}%]")
        print(f"  paired t-test p = {p:.4g}")
        print(f"  Cohen's d_z   = {dz:.3f}")
        return {"mean": mean_v, "ci_lo": ci_lo, "ci_hi": ci_hi, "p": p, "dz": dz}
    except ImportError:
        print(f"\n{label}: mean={mean_v:+.2f}%, std={std_v:.2f}% (scipy not available for CI/p)")
        return {"mean": mean_v}

fuel_stats = stats_of(fuel_sav_k, "FUEL SAVING (ours - ablation, % of ablation)")
dur_stats  = stats_of(dur_sav_k,  "TIME SAVING (ours - ablation, % of ablation)")

# ---------------------------------------------------------------------------
# Compare with summary JSON
# ---------------------------------------------------------------------------
if sum_json:
    print(f"\n{'='*60}")
    print(f"COMPARISON WITH SUMMARY JSON: {sum_json}")
    print(f"{'='*60}")
    with open(sum_json) as f:
        j = json.load(f)
    j_fuel_mean = j.get("fuel_sav_mean", "N/A")
    j_dur_mean  = j.get("dur_sav_mean",  "N/A")
    j_ci        = j.get("fuel_stats", {}).get("CI_95_pct", "N/A")
    j_p         = j.get("fuel_stats", {}).get("t_test_p", "N/A")
    j_dz        = j.get("fuel_stats", {}).get("cohen_dz", "N/A")
    print(f"  JSON fuel_sav_mean:  {j_fuel_mean}")
    print(f"  OUR  fuel_sav_mean:  {fuel_stats['mean']:+.2f}%")
    print(f"  JSON dur_sav_mean:   {j_dur_mean}")
    print(f"  OUR  dur_sav_mean:   {dur_stats['mean']:+.2f}%")
    print(f"  JSON CI_95_pct:      {j_ci}")
    if "ci_lo" in fuel_stats:
        print(f"  OUR  CI (fuel%):     [{fuel_stats['ci_lo']:+.2f}%, {fuel_stats['ci_hi']:+.2f}%]")
    print(f"  JSON p:              {j_p}")
    if "p" in fuel_stats:
        print(f"  OUR  p:              {fuel_stats['p']:.4g}")
    delta_fuel = abs(fuel_stats["mean"] - float(j_fuel_mean)) if j_fuel_mean != "N/A" else None
    if delta_fuel is not None:
        match = "MATCH" if delta_fuel < 0.01 else f"DIFFER by {delta_fuel:.4f}%"
        print(f"\n  Fuel mean match:     {match}")
    delta_dur = abs(dur_stats["mean"] - float(j_dur_mean)) if j_dur_mean != "N/A" else None
    if delta_dur is not None:
        match = "MATCH" if delta_dur < 0.01 else f"DIFFER by {delta_dur:.4f}%"
        print(f"  Time mean match:     {match}")

# ---------------------------------------------------------------------------
# Trip-11 sensitivity (Step 3D)
# ---------------------------------------------------------------------------
print(f"\n{'='*60}")
print("TRIP-11 SENSITIVITY (Step 3D)")
print(f"{'='*60}")

# Find worst-case ours fuel per seed (for trip-11 penalty)
seed_records_t11 = []
for s in seeds:
    all_trips = set(t for ss, t in ours_by if ss == s) | set(t for ss, t in base_by if ss == s)
    paired_keys = []
    for t in sorted(all_trips):
        o_row = ours_by.get((s, t))
        b_row = base_by.get((s, t))
        if (o_row and b_row and
                o_row.get("arrived", "").lower() == "true" and
                b_row.get("arrived", "").lower() == "true" and
                o_row.get("fuel_abs") not in ("", "None", None) and
                b_row.get("fuel_abs") not in ("", "None", None)):
            paired_keys.append(t)
    if not paired_keys:
        continue

    o_fuel_paired = [float(ours_by[(s,t)]["fuel_abs"]) for t in paired_keys]
    b_fuel_paired = [float(base_by[(s,t)]["fuel_abs"]) for t in paired_keys]
    o_dur_paired  = [float(ours_by[(s,t)]["duration"])  for t in paired_keys]
    b_dur_paired  = [float(base_by[(s,t)]["duration"])  for t in paired_keys]

    # Check if trip 11 is a non-arrover in ours
    t11_ours = ours_by.get((s, 11))
    t11_base = base_by.get((s, 11))
    t11_ours_arrived = t11_ours and t11_ours.get("arrived","").lower() == "true"
    t11_base_arrived = t11_base and t11_base.get("arrived","").lower() == "true"

    # (i) As-is: trip-11 excluded from paired (already in paired_keys or not)
    fuel_pct_excl = [100*(b-o)/b for o,b in zip(o_fuel_paired,b_fuel_paired) if b>0]
    dur_pct_excl  = [100*(b-o)/b for o,b in zip(o_dur_paired, b_dur_paired)  if b>0]
    mean_fuel_excl = sum(fuel_pct_excl)/len(fuel_pct_excl) if fuel_pct_excl else float("nan")
    mean_dur_excl  = sum(dur_pct_excl) /len(dur_pct_excl)  if dur_pct_excl  else float("nan")

    # (ii) Worst-case: if trip 11 was non-arriving in ours but arrived in ablation,
    # penalize: assign ours the max fuel/duration from paired ours trips
    max_ours_fuel = max(o_fuel_paired) if o_fuel_paired else 0
    max_ours_dur  = max(o_dur_paired)  if o_dur_paired  else 0

    t11_in_paired = 11 in paired_keys
    if not t11_ours_arrived and t11_base_arrived and t11_base and t11_base.get("fuel_abs") not in ("","None",None):
        # Trip 11 non-arrived in ours, arrived in ablation: add worst-case penalty
        b_t11_fuel = float(t11_base["fuel_abs"])
        b_t11_dur  = float(t11_base["duration"])
        # worst-case: ours used max_ours_fuel on this trip
        wc_fuel_pct = 100*(b_t11_fuel - max_ours_fuel)/b_t11_fuel if b_t11_fuel > 0 else 0
        wc_dur_pct  = 100*(b_t11_dur  - max_ours_dur) /b_t11_dur  if b_t11_dur  > 0 else 0
        fuel_pct_wc = fuel_pct_excl + [wc_fuel_pct]
        dur_pct_wc  = dur_pct_excl  + [wc_dur_pct]
    else:
        fuel_pct_wc = fuel_pct_excl
        dur_pct_wc  = dur_pct_excl

    mean_fuel_wc = sum(fuel_pct_wc)/len(fuel_pct_wc) if fuel_pct_wc else float("nan")
    mean_dur_wc  = sum(dur_pct_wc) /len(dur_pct_wc)  if dur_pct_wc  else float("nan")

    seed_records_t11.append({
        "seed": s,
        "excl_fuel": mean_fuel_excl, "excl_dur": mean_dur_excl,
        "wc_fuel":   mean_fuel_wc,   "wc_dur":   mean_dur_wc,
        "t11_ours_arrived": t11_ours_arrived,
    })
    print(f"  Seed {s}: trip11_ours_arrived={t11_ours_arrived}  "
          f"excl: fuel={mean_fuel_excl:+.2f}% dur={mean_dur_excl:+.2f}%  "
          f"wc: fuel={mean_fuel_wc:+.2f}% dur={mean_dur_wc:+.2f}%")

if seed_records_t11:
    excl_f = [r["excl_fuel"] for r in seed_records_t11]
    excl_d = [r["excl_dur"]  for r in seed_records_t11]
    wc_f   = [r["wc_fuel"]   for r in seed_records_t11]
    wc_d   = [r["wc_dur"]    for r in seed_records_t11]
    n = len(seed_records_t11)
    print(f"\n  (i)  Excluding trip-11:   fuel={sum(excl_f)/n:+.2f}%  time={sum(excl_d)/n:+.2f}%")
    print(f"  (ii) Worst-case trip-11:  fuel={sum(wc_f)/n:+.2f}%  time={sum(wc_d)/n:+.2f}%")
    try:
        from scipy import stats as sp
        _, p_excl_f = sp.ttest_1samp(excl_f, 0)
        _, p_wc_f   = sp.ttest_1samp(wc_f,   0)
        print(f"  one-sample t p (H0: mean=0): excl p={p_excl_f:.4g}, wc p={p_wc_f:.4g}")
        print(f"  Conclusion (i): CI excludes 0 = {p_excl_f < 0.05}")
        print(f"  Conclusion (ii): CI excludes 0 = {p_wc_f  < 0.05}")
    except ImportError:
        pass

# ---------------------------------------------------------------------------
# Mechanism check (Step 3E)
# ---------------------------------------------------------------------------
print(f"\n{'='*60}")
print("MECHANISM CHECK (Step 3E): |time%| vs |fuel%| per seed")
print(f"{'='*60}")
for r in seed_records:
    fuel_mag = abs(r["fuel_saving_pct"])
    time_mag = abs(r["dur_saving_pct"])
    dominant = "TIME" if time_mag >= fuel_mag else "FUEL"
    print(f"  Seed {r['seed']}: |fuel%|={fuel_mag:.2f}%  |time%|={time_mag:.2f}%  dominant={dominant}")
if seed_records:
    all_fuel_mag = [abs(r["fuel_saving_pct"]) for r in seed_records]
    all_time_mag = [abs(r["dur_saving_pct"])  for r in seed_records]
    n_time_dom = sum(1 for f, t in zip(all_fuel_mag, all_time_mag) if t >= f)
    print(f"\n  Across {len(seed_records)} seeds: time dominant in {n_time_dom} of {len(seed_records)}")
    overall_time_dom = sum(all_time_mag)/len(all_time_mag) >= sum(all_fuel_mag)/len(all_fuel_mag)
    if overall_time_dom:
        print("  HONEST STATEMENT: |time%| >= |fuel%| overall.")
        print("  => Dominant effect is congestion/route-quality avoidance.")
        print("     The degraded corridor is also a bottleneck, so both arms")
        print("     benefit from avoiding it but the time benefit exceeds the fuel benefit.")
        print("     We do NOT claim fuel-specific routing intelligence as the primary mechanism.")
    else:
        print("  |fuel%| > |time%| overall: fuel-specific benefit exceeds time benefit.")
        print("  => Fuel-specific routing does provide an incremental fuel benefit.")
