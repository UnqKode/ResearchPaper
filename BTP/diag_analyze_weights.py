"""
diag_analyze_weights.py — Round-6 D1/D2 offline analysis (no SUMO needed).

Run from BTP/:
    python diag_analyze_weights.py --arm ours_fuel --seed 1

Produces a DIAG_ROUND6.md section with findings for H1-H4.
"""
import argparse
import os
import sys

try:
    import pandas as pd
    import numpy as np
except ImportError:
    sys.exit("pandas and numpy required: conda activate ml && pip install pandas numpy")

_HERE = os.path.dirname(os.path.abspath(__file__))


# ---------------------------------------------------------------------------
def load_weights(arm_slug, seed):
    path = os.path.join(_HERE, f"diag_weights_{arm_slug}_seed{seed}.csv")
    if not os.path.exists(path):
        sys.exit(f"[ERROR] D1 file not found: {path}\n"
                 f"Run the diagnostic smoke first: "
                 f"python -m Simulation.compare_routing --diagnose --n 3 ...")
    df = pd.read_csv(path)
    print(f"[D1] Loaded {len(df)} edges from {path}")
    return df


def load_routes(arm_slug, seed):
    path = os.path.join(_HERE, f"diag_routes_{arm_slug}_seed{seed}.csv")
    if not os.path.exists(path):
        print(f"[D2] Route file not found: {path}  (D2 checks skipped)")
        return None
    df = pd.read_csv(path)
    print(f"[D2] Loaded {len(df)} ego route records from {path}")
    return df


# ---------------------------------------------------------------------------
def analyze_h1_h2(df):
    """H1: cold-start pricing bias; H2: uniform preseed poisons augtime."""
    print("\n" + "=" * 60)
    print("H1 / H2 — Weight-per-meter by baseline source")
    print("=" * 60)

    df = df.copy()
    df["weight_per_m"] = df["weight"] / df["length_m"].clip(lower=1.0)

    # Source breakdown
    src_counts = df["baseline_source"].value_counts()
    total = len(df)
    print(f"\nBaseline source distribution (n={total}):")
    for src, cnt in src_counts.items():
        print(f"  {src:<18}: {cnt:>5}  ({100*cnt/total:.1f}%)")

    # Mean weight/m by source — H1: preseed edges should look cheaper than observed
    grp = df.groupby("baseline_source")["weight_per_m"].agg(["mean", "median", "std", "count"])
    print("\nweight-per-meter stats by source:")
    print(grp.to_string())

    obs_med  = grp.loc["observed",  "median"] if "observed"  in grp.index else None
    pre_med  = grp.loc["preseed",   "median"] if "preseed"   in grp.index else None
    none_med = grp.loc["none",      "median"] if "none"      in grp.index else None

    print()
    if obs_med is not None and pre_med is not None:
        ratio = pre_med / obs_med
        verdict = "H1 CONFIRMED" if ratio < 0.9 else ("H1 BORDERLINE" if ratio < 1.0 else "H1 REFUTED")
        print(f"[H1] preseed_median/observed_median = {ratio:.3f}  → {verdict}")
        print(f"     (preseed={pre_med:.4f} s/m  vs  observed={obs_med:.4f} s/m)")
    else:
        print("[H1] Insufficient data — need both 'observed' and 'preseed' sources")

    # H2: F distribution — is F > 0 on preseed edges (meaning beta*F is inflating their weight)?
    print("\nH2 — F (fuel index) distribution by source:")
    f_grp = df.groupby("baseline_source")["F"].agg(["mean", "median", lambda x: (x > 0).mean()])
    f_grp.columns = ["mean_F", "median_F", "frac_F_pos"]
    print(f_grp.to_string())

    pre_F = f_grp.loc["preseed", "median_F"] if "preseed" in f_grp.index else None
    obs_F = f_grp.loc["observed", "median_F"] if "observed" in f_grp.index else None
    if pre_F is not None and obs_F is not None:
        h2_verdict = "H2 CONFIRMED" if pre_F > 0.01 else "H2 REFUTED"
        print(f"\n[H2] preseed edges median_F={pre_F:.4f}  observed median_F={obs_F:.4f}  → {h2_verdict}")

    # Multiplier distribution
    print("\nMultiplier distribution by source:")
    mult_grp = df.groupby("baseline_source")["multiplier"].agg(["mean", "median", "max"])
    print(mult_grp.to_string())

    return grp


def analyze_h3(df_routes, df_weights):
    """H3: corridor never engaged — initial routes bypass corridor."""
    print("\n" + "=" * 60)
    print("H3 — Corridor engagement from D2 route log")
    print("=" * 60)

    if df_routes is None:
        print("[H3] No D2 route data — skipping")
        return

    total = len(df_routes)
    crosses = df_routes["crosses_corridor"].sum()
    print(f"\nEgo routes: {total}  crosses_corridor: {crosses} ({100*crosses/max(total,1):.0f}%)")

    by_arm = df_routes.groupby("arm")["crosses_corridor"].agg(["sum", "count"])
    by_arm.columns = ["n_crossing", "n_total"]
    by_arm["pct"] = (100 * by_arm["n_crossing"] / by_arm["n_total"]).round(1)
    print(by_arm.to_string())

    if crosses == 0:
        print("\n[H3] CONFIRMED — zero egos cross the corridor at initial injection")
        print("     sumolib OD selection ≠ SUMO routing at insertion time")
    else:
        print(f"\n[H3] PARTIAL or REFUTED — {crosses}/{total} egos initially cross corridor")

    # Show corridor edges and route lengths
    print("\nSample routes with corridor_edges_in_route:")
    mask = df_routes["corridor_edges_in_route"].notna() & (df_routes["corridor_edges_in_route"] != "")
    if mask.any():
        print(df_routes[mask][["vid","arm","crosses_corridor","corridor_edges_in_route"]].to_string())
    else:
        print("  (none)")


def analyze_h4_transport(df_weights):
    """H4: transport — USING_LIBSUMO inferred from presence of data (actual value from stdout)."""
    print("\n" + "=" * 60)
    print("H4 — Transport audit (check stdout for [TRANSPORT] lines)")
    print("=" * 60)
    print("  Look for: [TRANSPORT] USING_LIBSUMO=True/False traci_module=...")
    print("  H4 CONFIRMED if USING_LIBSUMO=False for all arms")
    print("  Steps/s target: >= 30 from [PERF] lines in stderr")


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="Round-6 D1/D2 offline analysis")
    ap.add_argument("--arm", type=str, default="ours_fuel",
                    help="arm slug (underscores), e.g. ours_fuel, ours_augtime, ablation")
    ap.add_argument("--seed", type=int, default=1, help="seed number (default 1)")
    ap.add_argument("--all-arms", action="store_true",
                    help="analyze all three standard arms for this seed")
    args = ap.parse_args()

    arms = (["ours_fuel", "ours_augtime", "ablation"]
            if args.all_arms else [args.arm])

    for arm in arms:
        print(f"\n{'#'*70}")
        print(f"# Arm: {arm}  Seed: {args.seed}")
        print(f"{'#'*70}")
        df_w = load_weights(arm, args.seed)
        df_r = load_routes(arm, args.seed)
        analyze_h1_h2(df_w)
        analyze_h3(df_r, df_w)
        analyze_h4_transport(df_w)

    print("\n[DONE] Copy relevant sections into DIAG_ROUND6.md")


if __name__ == "__main__":
    main()
