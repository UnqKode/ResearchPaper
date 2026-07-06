"""
analyze_drift.py
================
Load drift_ours_seed1.csv and drift_ablation_seed1.csv, then compute:
  1. Global divergence: |veh_in_network(ours) - veh_in_network(ablation)|
     and as % of total background (~2400 vehicles).
  2. Per-edge divergence: fraction of monitored edges with differing counts,
     mean absolute difference.
  3. Corridor occupancy over time (corridor_count per arm).
  4. Trend: does divergence grow, stay flat, or stay ~0?

Usage:
    conda run -n ml python analyze_drift.py
    python analyze_drift.py  (if pandas is available)
"""

import csv
import os
import sys
from collections import defaultdict

OURS_CSV      = "drift_ours_seed1.csv"
ABLATION_CSV  = "drift_ablation_seed1.csv"
DEPART_START  = 21600.0   # ego departs at t=21600 s
TOTAL_BG_APPROX = 2400    # approximate total background vehicles at peak


def load_drift(path):
    """Return list of dicts, one per sampled time step."""
    rows = []
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            r = {}
            for k, v in row.items():
                try:
                    r[k] = float(v) if '.' in v else int(v)
                except (ValueError, TypeError):
                    r[k] = v
            rows.append(r)
    return rows


def monitored_edge_cols(rows):
    """Return list of edge-count column names (everything except fixed cols)."""
    if not rows:
        return []
    fixed = {"sim_time", "arm", "veh_in_network", "corridor_count"}
    return [k for k in rows[0].keys() if k not in fixed]


def main():
    for p in [OURS_CSV, ABLATION_CSV]:
        if not os.path.exists(p):
            print(f"ERROR: {p} not found. Run the simulation with --drift-log first.")
            sys.exit(1)

    ours_rows  = load_drift(OURS_CSV)
    abl_rows   = load_drift(ABLATION_CSV)

    print(f"Loaded {len(ours_rows)} ours rows, {len(abl_rows)} ablation rows")
    if not ours_rows or not abl_rows:
        print("ERROR: One or both drift CSV files are empty.")
        sys.exit(1)

    edge_cols = monitored_edge_cols(ours_rows)
    print(f"Monitored edges: {len(edge_cols)} columns")

    # Index by sim_time
    ours_by_t  = {r["sim_time"]: r for r in ours_rows}
    abl_by_t   = {r["sim_time"]: r for r in abl_rows}

    # Shared times
    shared_times = sorted(set(ours_by_t) & set(abl_by_t))
    print(f"Shared sample times: {len(shared_times)}")
    if not shared_times:
        print("No overlapping sample times — cannot compare.")
        sys.exit(1)

    # ----------------------------------------------------------------
    # 1. Global divergence at each shared time
    # ----------------------------------------------------------------
    print("\n" + "=" * 70)
    print("1. GLOBAL DIVERGENCE (veh_in_network ours vs ablation)")
    print("=" * 70)
    print(f"{'sim_time':>10}  {'ours_bg':>8}  {'abl_bg':>8}  "
          f"{'|diff|':>7}  {'diff_%':>7}  {'corr_ours':>10}  {'corr_abl':>10}")
    print("-" * 70)

    depart_diff = None
    arrive_diff = None
    arrive_diff_pct = None
    depart_corr_ours = None
    depart_corr_abl  = None
    arrive_corr_ours = None
    arrive_corr_abl  = None
    first_t = shared_times[0]
    last_t  = shared_times[-1]

    all_diffs = []
    for t in shared_times:
        o = ours_by_t[t]
        a = abl_by_t[t]
        diff = abs(o["veh_in_network"] - a["veh_in_network"])
        pct  = diff / TOTAL_BG_APPROX * 100.0
        all_diffs.append(diff)

        label = ""
        if t == first_t:
            depart_diff = diff
            depart_diff_pct = pct
            depart_corr_ours = o["corridor_count"]
            depart_corr_abl  = a["corridor_count"]
            label = " <-- EGO DEPART"
        if t == last_t:
            arrive_diff = diff
            arrive_diff_pct = pct
            arrive_corr_ours = o["corridor_count"]
            arrive_corr_abl  = a["corridor_count"]
            label = " <-- LAST SAMPLE"

        print(f"{t:>10.1f}  {o['veh_in_network']:>8}  {a['veh_in_network']:>8}  "
              f"{diff:>7}  {pct:>6.2f}%  {o['corridor_count']:>10}  "
              f"{a['corridor_count']:>10}{label}")

    # ----------------------------------------------------------------
    # 2. Per-edge divergence
    # ----------------------------------------------------------------
    print("\n" + "=" * 70)
    print("2. PER-EDGE DIVERGENCE (monitored edges away from corridor)")
    print("=" * 70)

    if edge_cols:
        # At depart time (first shared sample)
        t0, t1 = first_t, last_t
        o0, a0 = ours_by_t.get(t0), abl_by_t.get(t0)
        o1, a1 = ours_by_t.get(t1), abl_by_t.get(t1)

        def edge_divergence(o_row, a_row, cols):
            diffs = [abs((o_row.get(c) or 0) - (a_row.get(c) or 0)) for c in cols]
            n_differ = sum(1 for d in diffs if d > 0)
            frac = n_differ / len(cols) if cols else 0.0
            mean_abs = sum(diffs) / len(diffs) if diffs else 0.0
            return frac, mean_abs, diffs

        if o0 and a0:
            frac0, mean0, _ = edge_divergence(o0, a0, edge_cols)
            print(f"  At ego DEPART  (t={t0:.0f}s): "
                  f"{frac0*100:.1f}% of edges differ, mean |diff|={mean0:.2f} veh")
        if o1 and a1:
            frac1, mean1, _ = edge_divergence(o1, a1, edge_cols)
            print(f"  At last sample (t={t1:.0f}s): "
                  f"{frac1*100:.1f}% of edges differ, mean |diff|={mean1:.2f} veh")

        # Show top diverging edges at last sample
        if o1 and a1:
            diffs_by_edge = {c: abs((o1.get(c) or 0) - (a1.get(c) or 0))
                             for c in edge_cols}
            top5 = sorted(diffs_by_edge.items(), key=lambda x: x[1], reverse=True)[:5]
            print(f"\n  Top 5 diverging monitored edges at t={t1:.0f}s:")
            for eid, d in top5:
                print(f"    {eid}: |diff|={d} veh  "
                      f"(ours={o1.get(eid,0)}, abl={a1.get(eid,0)})")
    else:
        print("  No edge columns found in drift CSV.")

    # ----------------------------------------------------------------
    # 3. Corridor occupancy over time
    # ----------------------------------------------------------------
    print("\n" + "=" * 70)
    print("3. CORRIDOR OCCUPANCY OVER TIME")
    print("=" * 70)
    print(f"{'sim_time':>10}  {'corr_ours':>10}  {'corr_abl':>10}  "
          f"{'|diff|':>8}  notes")
    print("-" * 70)

    for t in shared_times:
        o = ours_by_t[t]
        a = abl_by_t[t]
        corr_diff = abs(o["corridor_count"] - a["corridor_count"])
        note = " <-- DEPART" if t == first_t else (" <-- END" if t == last_t else "")
        print(f"{t:>10.1f}  {o['corridor_count']:>10}  {a['corridor_count']:>10}  "
              f"{corr_diff:>8}  {note}")

    # ----------------------------------------------------------------
    # 4. Trend analysis — early vs late window
    # ----------------------------------------------------------------
    EARLY_WINDOW_S = 480.0   # first 480 sim-sec (what prior run measured)
    print("\n" + "=" * 70)
    print("4. TREND ANALYSIS  (early ≤480 s  vs  late >480 s from depart)")
    print("=" * 70)

    early_diffs = [all_diffs[i] for i, t in enumerate(shared_times)
                   if t - DEPART_START <= EARLY_WINDOW_S]
    late_diffs  = [all_diffs[i] for i, t in enumerate(shared_times)
                   if t - DEPART_START >  EARLY_WINDOW_S]

    overall_mean = sum(all_diffs) / len(all_diffs) if all_diffs else 0
    overall_max  = max(all_diffs) if all_diffs else 0

    print(f"  Shared points: {len(shared_times)}  "
          f"span: t={shared_times[0]:.0f}–{shared_times[-1]:.0f} s  "
          f"({shared_times[-1]-shared_times[0]:.0f} s window)")
    print(f"  Early window (≤480 s):  {len(early_diffs)} points  "
          f"mean={sum(early_diffs)/len(early_diffs):.1f} veh" if early_diffs
          else "  Early window: no points")
    print(f"  Late  window (>480 s):  {len(late_diffs)} points  "
          f"mean={sum(late_diffs)/len(late_diffs):.1f} veh" if late_diffs
          else "  Late  window (>480 s): no points yet")
    print(f"  Overall mean: {overall_mean:.1f} veh  |  max: {overall_max:.1f} veh")

    if len(all_diffs) >= 2:
        first_half = all_diffs[:len(all_diffs)//2]
        second_half = all_diffs[len(all_diffs)//2:]
        mean_first  = sum(first_half) / len(first_half)
        mean_second = sum(second_half) / len(second_half)
        if mean_second > mean_first * 1.5:
            trend = "GROWING (2nd half >1.5x 1st half mean)"
        elif mean_second < mean_first * 0.67:
            trend = "SHRINKING"
        else:
            trend = "FLAT / STABLE"
        print(f"  Trend (half-split): {trend}")
    else:
        print("  Not enough samples for trend analysis.")

    # ----------------------------------------------------------------
    # 5. Plain-language verdict
    # ----------------------------------------------------------------
    print("\n" + "=" * 70)
    print("5. VERDICT")
    print("=" * 70)

    if depart_diff is None or arrive_diff is None:
        print("  Cannot determine verdict — missing depart or arrive data.")
        return

    depart_pct_str = f"{depart_diff / TOTAL_BG_APPROX * 100:.2f}%"
    arrive_pct_str = f"{arrive_diff / TOTAL_BG_APPROX * 100:.2f}%"

    print(f"  Global divergence at ego depart (t={first_t:.0f}s): "
          f"{depart_diff} veh = {depart_pct_str} of ~{TOTAL_BG_APPROX}")
    print(f"  Global divergence at last sample (t={last_t:.0f}s): "
          f"{arrive_diff} veh = {arrive_pct_str} of ~{TOTAL_BG_APPROX}")
    print(f"  Corridor (ours vs ablation) at depart: "
          f"{depart_corr_ours} vs {depart_corr_abl}")
    print(f"  Corridor (ours vs ablation) at end:    "
          f"{arrive_corr_ours} vs {arrive_corr_abl}")

    max_pct = max(depart_diff, arrive_diff) / TOTAL_BG_APPROX * 100.0

    if max_pct < 2.0:
        verdict = ("VALID PAIRED COMPARISON. "
                   f"Background divergence stays under 2% (max {max_pct:.2f}%). "
                   "The two worlds are effectively identical away from the ego. "
                   "Any difference in ego fuel/time is attributable to the routing "
                   "policy, not to differing background conditions.")
    else:
        verdict = (f"POSSIBLE LIMITATION. "
                   f"Background divergence reaches {max_pct:.2f}%. "
                   "The background traffic reacts to the ego's different route "
                   "enough to create materially different conditions in the two arms. "
                   "This should be reported as a limitation that scales with trip "
                   "length and departure time.")

    print(f"\n  >> {verdict}")


if __name__ == "__main__":
    main()
