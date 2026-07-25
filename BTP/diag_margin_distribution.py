"""D4 offline analysis: mine [FUEL_HYST] lines from captured campaign output.

Usage:
    python diag_margin_distribution.py <log_file> [<log_file> ...]

Each log file should be a captured stdout/stderr from a diagnostic or campaign run
containing lines like:
    [FUEL_HYST] vehicle=ego_0 action=KEEP cost_cur=12345.0 cost_new=12300.0
                threshold=11727.8 margin=0.365%
    [FUEL_HYST] vehicle=ego_1 action=REROUTE cost_cur=9800.0 cost_new=9200.0
                saving=6.1% threshold=9310.0

Also parses pre-summarised [D4_MARGINS] lines if raw [FUEL_HYST] are absent:
    [D4_MARGINS] arm=ours-fuel seed=1 n=47 p50=0.123% p90=2.456% p99=5.0% max=8.1%

Reports per-arm p50/p90/p99/max margins and SWITCH vs KEEP counts.
The p90 value is the recommended starting δ for Fix S (hysteresis threshold).

No SUMO dependency — pure Python stdlib only.
"""
from __future__ import annotations
import re, sys, collections, pathlib

_HYST_RE = re.compile(
    r"\[FUEL_HYST\].*?action=(?P<action>\w+)"
    r".*?cost_cur=(?P<cost_cur>[\d.]+)"
    r".*?cost_new=(?P<cost_new>[\d.]+)"
)
_D4_RE = re.compile(
    r"\[D4_MARGINS\] arm=(?P<arm>\S+) seed=(?P<seed>\S+) n=(?P<n>\d+)"
    r" p50=(?P<p50>[\d.]+)% p90=(?P<p90>[\d.]+)%"
    r"(?: p99=(?P<p99>[\d.]+)%)?"
    r" max=(?P<max>[\d.]+)%"
)
_ARM_RE = re.compile(r"\[(?P<arm>ours-fuel|ours-augtime|ablation)\]")


def _parse_arm_from_context(lines, idx):
    for i in range(max(0, idx - 200), idx):
        m = _ARM_RE.search(lines[i])
        if m:
            return m.group("arm")
    return "unknown"


def parse_files(paths):
    """Return (raw_records, d4_summaries) where:
    raw_records:  list of {arm, action, cost_cur, cost_new, margin_pct}
    d4_summaries: list of {arm, seed, n, p50, p90, p99, max}
    """
    raw, d4 = [], []
    for p in paths:
        text = pathlib.Path(p).read_text(encoding="utf-8", errors="replace")
        lines = text.splitlines()

        for idx, line in enumerate(lines):
            m = _HYST_RE.search(line)
            if m:
                action   = m.group("action")
                cost_cur = float(m.group("cost_cur"))
                cost_new = float(m.group("cost_new"))
                if cost_cur <= 0:
                    continue
                margin = (cost_cur - cost_new) / cost_cur * 100.0
                arm = _parse_arm_from_context(lines, idx)
                raw.append({"arm": arm, "action": action,
                            "cost_cur": cost_cur, "cost_new": cost_new,
                            "margin_pct": margin})
                continue

            m2 = _D4_RE.search(line)
            if m2:
                d4.append({
                    "arm": m2.group("arm"),
                    "seed": m2.group("seed"),
                    "n":    int(m2.group("n")),
                    "p50":  float(m2.group("p50")),
                    "p90":  float(m2.group("p90")),
                    "p99":  float(m2.group("p99")) if m2.group("p99") else None,
                    "max":  float(m2.group("max")),
                })

    return raw, d4


def _percentile(sorted_vals, q):
    if not sorted_vals:
        return float("nan")
    idx = min(int(q * len(sorted_vals)), len(sorted_vals) - 1)
    return sorted_vals[idx]


def report_raw(raw_records):
    by_arm = collections.defaultdict(list)
    for r in raw_records:
        by_arm[r["arm"]].append(r)

    print(f"\n=== D4 MARGIN DISTRIBUTION (from {len(raw_records)} [FUEL_HYST] evaluations) ===")
    all_margins = []
    for arm in sorted(by_arm):
        recs = by_arm[arm]
        keeps   = [r for r in recs if r["action"] == "KEEP"]
        reroutes = [r for r in recs if r["action"] == "REROUTE"]
        margins  = sorted(r["margin_pct"] for r in keeps)  # rejected margins (KEEP actions)
        all_margins.extend(margins)
        print(f"\n  Arm: {arm}")
        print(f"    evaluations: {len(recs)}  SWITCH={len(reroutes)}  KEEP={len(keeps)}")
        if margins:
            print(f"    KEEP margins (improvement that was rejected):")
            print(f"      p50={_percentile(margins, 0.5):.3f}%  "
                  f"p90={_percentile(margins, 0.9):.3f}%  "
                  f"p99={_percentile(margins, 0.99):.3f}%  "
                  f"max={margins[-1]:.3f}%")
        if reroutes:
            savings = sorted(r["margin_pct"] for r in reroutes)
            print(f"    SWITCH savings:")
            print(f"      p50={_percentile(savings, 0.5):.3f}%  "
                  f"p90={_percentile(savings, 0.9):.3f}%  "
                  f"max={savings[-1]:.3f}%")
        if margins:
            p90 = _percentile(margins, 0.9)
            print(f"    >> Recommended Fix S δ (p90 of rejected margins): {p90:.2f}%")

    if all_margins:
        all_margins.sort()
        print(f"\n  ALL ARMS combined KEEP margins ({len(all_margins)} evaluations):")
        print(f"    p50={_percentile(all_margins, 0.5):.3f}%  "
              f"p90={_percentile(all_margins, 0.9):.3f}%  "
              f"p99={_percentile(all_margins, 0.99):.3f}%  "
              f"max={all_margins[-1]:.3f}%")
        print(f"    >> Global Fix S δ recommendation: {_percentile(all_margins, 0.9):.2f}%")


def report_d4_summaries(d4):
    print(f"\n=== D4 SUMMARIES (from {len(d4)} [D4_MARGINS] lines) ===")
    for s in d4:
        p99_str = f"p99={s['p99']:.3f}%  " if s["p99"] is not None else ""
        print(f"  arm={s['arm']} seed={s['seed']} n={s['n']}  "
              f"p50={s['p50']:.3f}%  p90={s['p90']:.3f}%  "
              f"{p99_str}max={s['max']:.3f}%")
        print(f"    >> Fix S δ (p90): {s['p90']:.2f}%")


def main():
    if len(sys.argv) < 2:
        print("Usage: python diag_margin_distribution.py <log_file> [<log_file> ...]")
        sys.exit(1)

    raw, d4 = parse_files(sys.argv[1:])

    if raw:
        report_raw(raw)
    elif d4:
        print("\n[NOTE] No raw [FUEL_HYST] lines found; reporting pre-summarised [D4_MARGINS].")
        report_d4_summaries(d4)
    else:
        print("\n[WARN] No [FUEL_HYST] or [D4_MARGINS] lines found in the provided files.")
        print("       Round-5 campaign stdout was not archived. D4 requires the new")
        print("       diagnostic run's output (--diagnose with --diagnose-exit-at 21900).")
        print("       Pass the captured log file once the diagnostic run completes.")
        sys.exit(0)


if __name__ == "__main__":
    main()
