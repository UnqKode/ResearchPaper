"""
compare_routing.py
==================
A/B-compare OUR fuel-aware routing against SUMO's own routing on the SAME set of
origin/destination points.

Design
------
1. Build ONE set of N (origin, dest) edge pairs from the network (seeded, so it
   is reproducible and identical for both scenarios).
2. Scenario A: launch SUMO (seed S), run ONE ego vehicle through every OD pair
   in sequence with ego_routing="sumo".
3. Scenario B: launch SUMO again with the SAME seed S and the SAME OD list, run
   the ego through every pair with ego_routing="ours".
4. For every pair where the ego arrived in BOTH scenarios, compare trip time and
   fuel; report means, per-trip % savings, win counts, and (if SciPy is present)
   a paired significance test.

Because both runs share the traffic seed and the OD list, the only systematic
difference is who routes the ego -- so the per-pair delta is attributable to the
routing algorithm.

CAVEAT: a single ego runs the pairs back-to-back, so the start time of pair k
drifts slightly between the two scenarios (faster routing reaches pair k sooner
and meets slightly different traffic). With a shared seed the background stream
is statistically identical, so this averages out over N pairs; if you need
strict per-pair traffic identity, schedule fixed ego departure times instead.

Intended launch commands (run from the project root BTP/):
    python -m Simulation.compare_routing           # defaults: 100 pairs
    python -m Simulation.compare_routing --n 50 --seed 7

Do NOT run as: cd Simulation && python compare_routing.py
    -- that breaks the Simulation.* imports and root-relative file paths.

For the single-trip path use:
    python main.py          (from BTP/)
    python -m main          (from BTP/)
"""

import os
import sys
import csv
import argparse
import random
import xml.etree.ElementTree as ET

# --- locate SUMO tools (same bootstrap as main.py) ---
if "SUMO_HOME" in os.environ:
    sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))
else:
    sys.exit("Error: Please declare the environment variable 'SUMO_HOME'")

import traci
import sumolib

from Simulation.simulate import Simulation


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
# Resolve paths relative to the repo root (MoSTScenario/), which is two
# directories above the Simulation/ folder where this file lives:
#   __file__     ->  Simulation/compare_routing.py
#   _HERE        ->  Simulation/
#   _BTP         ->  BTP/
#   PROJECT_ROOT ->  MoSTScenario/   <-- scenario files live here
# Launch as:  python -m Simulation.compare_routing   (from BTP/)
_HERE        = os.path.dirname(os.path.abspath(__file__))   # Simulation/
_BTP         = os.path.dirname(_HERE)                        # BTP/
PROJECT_ROOT = os.path.dirname(_BTP)                        # MoSTScenario/
CONFIG_FILE  = os.path.join(PROJECT_ROOT, "scenario", "most.sumocfg")
NET_FILE     = os.path.join(PROJECT_ROOT, "scenario", "in", "most.net.xml")
EGO_TYPE     = "DEFAULT_VEHTYPE"      # must be a vType that exists in MoST and has fuel
SUMO_BIN     = "sumo"                 # headless; use "sumo-gui" only to eyeball one run

REROUTE_INTERVAL = 30
ALPHA, BETA, GAMMA = 1.0, 0.8, 1.5
PER_TRIP_TIMEOUT = 3000              # steps before a trip is declared "did not arrive"
WARMUP_STEPS     = 300               # Q2: warm-up steps before first ego trip


# ---------------------------------------------------------------------------
# 1. Build the OD set once (offline, reproducible)
# ---------------------------------------------------------------------------
def generate_od_pairs(net_file, n, seed, vclass="passenger", min_len=30.0):
    """
    Pick n reachable (origin, dest) edge-ID pairs from the network using sumolib
    offline. Reachability is checked with net.getShortestPath so we never hand a
    disconnected pair to the simulation. Deterministic given `seed`.
    """
    print(f"Building {n} OD pairs from {net_file} (seed={seed}) ...")
    net = sumolib.net.readNet(net_file)

    candidates = [
        e for e in net.getEdges()
        if (not e.isSpecial())
        and e.allows(vclass)
        and e.getLength() >= min_len
    ]
    if len(candidates) < 2:
        sys.exit("Not enough routable edges to build OD pairs.")

    rng = random.Random(seed)
    pairs = []
    seen = set()
    attempts = 0
    max_attempts = n * 200

    while len(pairs) < n and attempts < max_attempts:
        attempts += 1
        a = rng.choice(candidates)
        b = rng.choice(candidates)
        if a.getID() == b.getID():
            continue
        key = (a.getID(), b.getID())
        if key in seen:
            continue
        seen.add(key)
        try:
            path, cost = net.getShortestPath(a, b, vClass=vclass)
        except Exception:
            path, cost = None, None
        if path and len(path) >= 1 and cost is not None and cost < float("inf"):
            pairs.append(key)

    if len(pairs) < n:
        print(f"  WARNING: only found {len(pairs)} routable pairs "
              f"(requested {n}) after {attempts} attempts.")
    else:
        print(f"  Got {len(pairs)} routable pairs in {attempts} attempts.")
    return pairs


# ---------------------------------------------------------------------------
# 2. Run one scenario (one routing mode) over the OD set
# ---------------------------------------------------------------------------
def run_scenario(mode, od_list, traffic_seed, tag, warmup_steps=WARMUP_STEPS):
    """
    Launch SUMO and run the ego through every OD pair under `mode`
    ("ours" or "sumo"). Returns (live_results, tripinfo_by_id).
    warmup_steps is forwarded to sim.run_od_campaign; both scenarios receive
    the same value so A/B fairness is preserved.
    """
    tripinfo_path = f"tripinfo_{tag}.xml"
    sumo_cmd = [
        SUMO_BIN, "-c", CONFIG_FILE,
        "--seed", str(traffic_seed),
        "--scale","3",
        "--no-step-log", "true",
        "--no-warnings", "true",
        "--time-to-teleport", "-1",          # don't teleport stuck cars; keeps timing honest
        "--end", "100000",                    # extend horizon so all trips fit
        "--tripinfo-output", tripinfo_path,
        # Background traffic uses SUMO dynamic rerouting in BOTH scenarios so it
        # behaves identically; only the ego's controller differs by `mode`.
        "--device.rerouting.probability", "1",
        "--device.rerouting.period", str(REROUTE_INTERVAL),
    ]

    print(f"\n=== Scenario '{tag}' (ego_routing={mode}, seed={traffic_seed}) ===")
    started = False
    live_results = []
    try:
        traci.start(sumo_cmd)
        started = True
        sim = Simulation(
            net_file=NET_FILE,
            reroute_interval=REROUTE_INTERVAL,
            alpha=ALPHA, beta=BETA, gamma=GAMMA,
            ego_type=EGO_TYPE,
            ego_routing=mode,
            ego_od_list=od_list,
        )
        live_results = sim.run_od_campaign(per_trip_timeout=PER_TRIP_TIMEOUT,
                                           warmup_steps=warmup_steps)
    except traci.FatalTraCIError as e:
        print(f"TraCI error in scenario '{tag}': {e}")
    finally:
        if started:
            traci.close()
            sys.stdout.flush()

    tripinfo = parse_tripinfo(tripinfo_path)
    return live_results, tripinfo


def parse_tripinfo(path):
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
    return out


# ---------------------------------------------------------------------------
# 3. Merge live + tripinfo, prefer tripinfo when available
# ---------------------------------------------------------------------------
def merge(live_results, tripinfo):
    """
    For each trip produce a clean record. Time and fuel come from tripinfo when
    present (SUMO's own authoritative accounting), else from the live integration.
    The ego id for trip k is 'ego_k'.
    """
    merged = []
    for r in live_results:
        vid = f"ego_{r['trip']}"
        ti = tripinfo.get(vid, {})
        duration = ti.get("duration") if ti.get("duration") is not None else r["duration_s"]
        fuel = ti.get("fuel_mg") if ti.get("fuel_mg") is not None else r["fuel_mg"]
        arrived = r["arrived"] and (duration is not None)
        merged.append({
            "trip": r["trip"],
            "origin": r["origin"],
            "dest": r["dest"],
            "arrived": arrived,
            "duration_s": duration if arrived else None,
            "fuel_mg": fuel if arrived else None,
            "reroutes": r["reroutes"],
        })
    return merged


# ---------------------------------------------------------------------------
# 4. Compare and report
# ---------------------------------------------------------------------------
def _mean(xs):
    return sum(xs) / len(xs) if xs else float("nan")


def _median(xs):
    if not xs:
        return float("nan")
    s = sorted(xs)
    m = len(s) // 2
    return s[m] if len(s) % 2 else (s[m - 1] + s[m]) / 2.0


def compare(sumo_m, ours_m, csv_path="comparison_results.csv"):
    # Index by trip number.
    sumo_by = {r["trip"]: r for r in sumo_m}
    ours_by = {r["trip"]: r for r in ours_m}
    trips = sorted(set(sumo_by) | set(ours_by))

    # Write the full per-trip table first (every trip, arrived or not).
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["trip", "origin", "dest",
                    "sumo_arrived", "sumo_time_s", "sumo_fuel_mg",
                    "ours_arrived", "ours_time_s", "ours_fuel_mg",
                    "ours_reroutes"])
        for t in trips:
            s = sumo_by.get(t, {})
            o = ours_by.get(t, {})
            w.writerow([
                t,
                s.get("origin", o.get("origin", "")),
                s.get("dest", o.get("dest", "")),
                s.get("arrived"), s.get("duration_s"), s.get("fuel_mg"),
                o.get("arrived"), o.get("duration_s"), o.get("fuel_mg"),
                o.get("reroutes"),
            ])

    # Keep only pairs where BOTH arrived -- those are the comparable trips.
    paired = []
    for t in trips:
        s, o = sumo_by.get(t), ours_by.get(t)
        if s and o and s["arrived"] and o["arrived"]:
            paired.append((t, s, o))

    n_total = len(trips)
    n_paired = len(paired)

    print("\n" + "=" * 64)
    print("RESULTS: our routing vs SUMO routing")
    print("=" * 64)
    print(f"OD pairs attempted:           {n_total}")
    print(f"Comparable (both arrived):    {n_paired}")
    if n_paired == 0:
        print("No comparable trips -- cannot compare. Check ego injection / vType / fuel model.")
        print(f"Per-trip table written to {csv_path}")
        return

    sumo_t = [s["duration_s"] for _, s, _ in paired]
    ours_t = [o["duration_s"] for _, _, o in paired]
    sumo_f = [s["fuel_mg"]    for _, s, _ in paired]
    ours_f = [o["fuel_mg"]    for _, _, o in paired]

    # Per-trip percentage savings of ours vs sumo (positive = ours better).
    time_sav = [100.0 * (s - o) / s for s, o in zip(sumo_t, ours_t) if s > 0]
    fuel_sav = [100.0 * (s - o) / s for s, o in zip(sumo_f, ours_f) if s > 0]

    time_wins = sum(1 for s, o in zip(sumo_t, ours_t) if o < s)
    fuel_wins = sum(1 for s, o in zip(sumo_f, ours_f) if o < s)

    def block(name, sumo_vals, ours_vals, sav):
        print(f"\n-- {name} --")
        print(f"  SUMO   mean / median : {_mean(sumo_vals):10.1f} / {_median(sumo_vals):10.1f}")
        print(f"  OURS   mean / median : {_mean(ours_vals):10.1f} / {_median(ours_vals):10.1f}")
        print(f"  total  SUMO  / OURS  : {sum(sumo_vals):10.1f} / {sum(ours_vals):10.1f}")
        print(f"  mean per-trip saving : {_mean(sav):+.2f}%   (positive = ours better)")

    block("TIME (s)", sumo_t, ours_t, time_sav)
    print(f"  ours faster on        : {time_wins}/{n_paired} trips "
          f"({100.0*time_wins/n_paired:.0f}%)")

    block("FUEL (mg)", sumo_f, ours_f, fuel_sav)
    print(f"  ours used less fuel on: {fuel_wins}/{n_paired} trips "
          f"({100.0*fuel_wins/n_paired:.0f}%)")

    # Optional paired significance test (runs are paired by OD pair).
    try:
        from scipy import stats
        td = [o - s for s, o in zip(sumo_t, ours_t)]
        fd = [o - s for s, o in zip(sumo_f, ours_f)]
        t_t, t_p = stats.ttest_rel(ours_t, sumo_t)
        f_t, f_p = stats.ttest_rel(ours_f, sumo_f)
        print("\n-- paired t-test (ours - sumo) --")
        print(f"  time: t={t_t:+.3f}  p={t_p:.4g}")
        print(f"  fuel: t={f_t:+.3f}  p={f_p:.4g}")
        try:
            _, wt_p = stats.wilcoxon(td)
            _, wf_p = stats.wilcoxon(fd)
            print(f"  Wilcoxon p  time={wt_p:.4g}  fuel={wf_p:.4g}")
        except Exception:
            pass
    except ImportError:
        print("\n(install scipy for paired t-test / Wilcoxon significance values)")

    # One-line verdict.
    mf = _mean(fuel_sav)
    mt = _mean(time_sav)
    verb_f = "less" if mf >= 0 else "MORE"
    verb_t = "less" if mt >= 0 else "MORE"
    print("\n-- verdict --")
    print(f"  On average our routing used {abs(mf):.1f}% {verb_f} fuel and took "
          f"{abs(mt):.1f}% {verb_t} time than SUMO's routing.")
    print(f"\nPer-trip table written to {csv_path}")


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=100, help="number of OD pairs")
    ap.add_argument("--od-seed", type=int, default=42, help="seed for OD generation")
    ap.add_argument("--traffic-seed", type=int, default=1, help="SUMO traffic seed (same for both runs)")
    args = ap.parse_args()

    od_list = generate_od_pairs(NET_FILE, n=args.n, seed=args.od_seed)
    if not od_list:
        sys.exit("No OD pairs generated; aborting.")

    # Scenario A: SUMO routes the ego.
    sumo_live, sumo_ti = run_scenario("sumo", od_list, args.traffic_seed, tag="sumo")
    # Scenario B: our algorithm routes the ego (same seed, same OD list).
    ours_live, ours_ti = run_scenario("ours", od_list, args.traffic_seed, tag="ours")

    sumo_m = merge(sumo_live, sumo_ti)
    ours_m = merge(ours_live, ours_ti)
    compare(sumo_m, ours_m)


if __name__ == "__main__":
    main()
