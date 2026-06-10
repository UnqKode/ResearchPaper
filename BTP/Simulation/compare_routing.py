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
import io
import time
import multiprocessing
import concurrent.futures
from contextlib import redirect_stdout
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
PER_TRIP_TIMEOUT = 10000     # Maximum simulation steps to wait for a single trip is declared "did not arrive"
WARMUP_STEPS     = 300               # Q2: warm-up steps before first ego trip
PROGRESS_LOG_EVERY = 1               # flush the live per-trip log every N trips (1 = every trip)

# --- SUMO demand / teleport knobs (HP-2). Exposed as CLI flags; these are just
# the defaults. NOTE: --scale 3 triples demand and can saturate the MoST network;
# combined with --time-to-teleport -1 (never teleport) a gridlock cluster can
# persist so every remaining ego trip times out (hours of wasted compute). If you
# see repeated TIMEOUTs, drop the scale or set a finite teleport threshold.
DEFAULT_SCALE    = 3.0               # background demand multiplier
DEFAULT_TELEPORT = -1                # seconds stuck before teleport (-1 = never)
WORKER_TIMEOUT_S = 7200              # HP-1: hard wall-clock cap per parallel scenario
TRIPINFO_FLUSH_WAIT_S = 10.0         # HP-5: max seconds to wait for SUMO to flush tripinfo

# --- Checkpointed Campaign Knobs ---
CHECKPOINT_DIR   = os.path.join(_BTP, "checkpoints")
CKPT_SPACING     = 240               # background steps between checkpoints
REWARM_STEPS     = 240               # background steps to populate RSUs after loadState


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
def _free_port():
    """Pick an OS-assigned free TCP port (CR-1 hardening: give each parallel
    SUMO an explicit, distinct port so two workers can never race for one)."""
    import socket
    with socket.socket() as s:
        s.bind(("", 0))
        return s.getsockname()[1]


def build_checkpoints(n_pairs, seed, scale, teleport, spacing=CKPT_SPACING, warmup=REWARM_STEPS):
    """
    Phase A: Run a reference SUMO instance (no ego) to generate exactly N checkpoints.
    Checkpoints ensure drift-free A/B testing because both algorithms start each
    trip from a byte-identical background traffic state.
    """
    os.makedirs(CHECKPOINT_DIR, exist_ok=True)
    paths = []
    
    # Pre-clean stale checkpoints from previous runs with this seed
    for f in os.listdir(CHECKPOINT_DIR):
        if f.startswith(f"ckpt_seed{seed}_"):
            os.remove(os.path.join(CHECKPOINT_DIR, f))

    print(f"\n[Phase A] Building {n_pairs} checkpoints for seed {seed} ...")
    port = _free_port()
    # add 86400 (24h) to ensure the end time exceeds the sumocfg begin time (e.g. 28800)
    horizon_s = 86400 + int((warmup + n_pairs * spacing) * 0.25) + 3600
    cmd = [
        SUMO_BIN, "-c", CONFIG_FILE,
        "--seed", str(seed), "--scale", str(scale),
        "--time-to-teleport", str(teleport),
        "--no-step-log", "true", "--no-warnings", "true",
        "--end", str(horizon_s),
        "--output-prefix", "",
        "--device.rerouting.probability", "0",
        "--route-steps", "0"
    ]
    
    traci.start(cmd, port=port)
    try:
        TARGET_START_TIME = 21600  # 06:00; commercial demand starts at 18006, so let it ramp
        while traci.simulation.getTime() < TARGET_START_TIME:
            traci.simulationStep()
        for _ in range(warmup):      # small extra settle
            traci.simulationStep()
            
        for i in range(n_pairs):
            path = os.path.abspath(os.path.join(CHECKPOINT_DIR, f"ckpt_seed{seed}_{i}.xml"))
            traci.simulation.saveState(path)
            paths.append(path)
            if i % 10 == 0 or i == n_pairs - 1:
                print(f"  saved checkpoint {i+1}/{n_pairs}")
            
            for _ in range(spacing):
                traci.simulationStep()
    finally:
        try:
            traci.close()
        except Exception:
            pass
    print(f"[Phase A] Done. Saved {len(paths)} checkpoints.")
    return paths


def run_scenario(mode, od_list, traffic_seed, tag, warmup_steps=WARMUP_STEPS,
                 log_every=PROGRESS_LOG_EVERY, scale=DEFAULT_SCALE,
                 teleport=DEFAULT_TELEPORT, checkpoints=None, rewarm_steps=REWARM_STEPS):
    """
    Launch SUMO and run the ego through every OD pair under `mode`
    ("ours" or "sumo"). Returns (live_results, tripinfo_by_id).

    OUTPUT ISOLATION (fixes CR-2 + the output-prefix tripinfo bug + CR-3):
    most.sumocfg sets <output-prefix value="most."/>, which SUMO prepends to
    command-line outputs too -- so a plain --tripinfo-output tripinfo.xml would
    silently become most.tripinfo.xml and never be found, and BOTH parallel
    SUMO instances would write the same most.sim.log. We override the prefix to
    "<tag>." so every output is per-scenario and predictably named, then delete
    any stale file from a previous run before launching.

    A live per-trip CSV (progress_<tag>.csv) is written and flushed as each
    trip finishes, so progress is visible on disk even while the run is going.
    """
    out_prefix    = f"{tag}."
    tripinfo_path = f"{out_prefix}tripinfo.xml"   # actual file after output-prefix
    progress_path = f"progress_{tag}.csv"

    # CR-3: never parse a stale tripinfo from an interrupted earlier run.
    try:
        if os.path.exists(tripinfo_path):
            os.remove(tripinfo_path)
    except OSError:
        pass

    # LP-2: size --end to the campaign instead of a fixed 100000 that can fire
    # mid-run for large N. step-length is 0.25 s, so steps * 0.25 = sim-seconds;
    # add the scenario begin time and a generous buffer.
    horizon_s = 86400 + int((warmup_steps + len(od_list) * PER_TRIP_TIMEOUT) * 0.25) + 20000

    port = _free_port()
    sumo_cmd = [
        SUMO_BIN, "-c", CONFIG_FILE,
        "--seed", str(traffic_seed),
        "--scale", str(scale),                # HP-2: configurable demand
        "--no-step-log", "true",
        "--no-warnings", "true",
        "--time-to-teleport", str(teleport),  # HP-2: configurable teleport threshold
        "--end", str(horizon_s),              # LP-2: scales with the campaign
        "--output-prefix", out_prefix,        # CR-2/CR-3: per-scenario output names
        "--tripinfo-output", "tripinfo.xml",  # -> "<tag>.tripinfo.xml"
        "--log", "sim.log",                   # -> "<tag>.sim.log" (no clobber)
        "--error-log", "errors.log",          # -> "<tag>.errors.log"
        # Background traffic uses SUMO dynamic rerouting in BOTH scenarios so it
        # behaves identically; only the ego's controller differs by `mode`.
        "--device.rerouting.probability", "0",
        "--device.rerouting.period", str(REROUTE_INTERVAL),
        "--route-steps", "0"
    ]

    print(f"\n=== Scenario '{tag}' (ego_routing={mode}, seed={traffic_seed}, "
          f"scale={scale}, teleport={teleport}, port={port}) ===")
    started = False
    live_results = []
    try:
        traci.start(sumo_cmd, port=port)
        started = True
        sim = Simulation(
            net_file=NET_FILE,
            reroute_interval=REROUTE_INTERVAL,
            alpha=ALPHA, beta=BETA, gamma=GAMMA,
            ego_type=EGO_TYPE,
            ego_routing=mode,
            ego_od_list=od_list,
        )
        if checkpoints is not None:
            live_results = sim.run_checkpointed_campaign(
                checkpoints, per_trip_timeout=PER_TRIP_TIMEOUT,
                rewarm_steps=rewarm_steps, seed=traffic_seed,
                verbose=True, progress_log_path=progress_path, log_every=log_every
            )
        else:
            live_results = sim.run_od_campaign(per_trip_timeout=PER_TRIP_TIMEOUT,
                                               warmup_steps=warmup_steps,
                                               progress_log_path=progress_path,
                                               log_every=log_every)
    except traci.FatalTraCIError as e:
        print(f"TraCI error in scenario '{tag}': {e}")
    finally:
        # CR-4: SUMO may already be dead; closing then can raise. Guard it so a
        # secondary error can't mask the primary one or leak the subprocess.
        if started:
            try:
                traci.close()
            except Exception as e:
                print(f"  (traci.close after scenario '{tag}' raised: {e!r})")
            sys.stdout.flush()

    # HP-5: SUMO writes tripinfo asynchronously during shutdown; on Windows/NTFS
    # and networked/OneDrive paths the file can lag. Wait briefly for it to land.
    deadline = time.time() + TRIPINFO_FLUSH_WAIT_S
    while time.time() < deadline:
        if os.path.exists(tripinfo_path) and os.path.getsize(tripinfo_path) > 0:
            break
        time.sleep(0.25)

    tripinfo = parse_tripinfo(tripinfo_path)
    return live_results, tripinfo


def parse_tripinfo(path):
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


def compare(sumo_m, ours_m, csv_path=None, run_meta=None):
    # MP-3/LP-3: don't silently overwrite previous results. Stamp the filename and
    # embed the run parameters so a CSV is self-describing and reproducible.
    if csv_path is None:
        stamp = time.strftime("%Y%m%d_%H%M%S")
        sd = f"_seed{run_meta['traffic_seed']}_n{run_meta['n']}" if run_meta else ""
        csv_path = f"comparison_{stamp}{sd}.csv"

    # Index by trip number.
    sumo_by = {r["trip"]: r for r in sumo_m}
    ours_by = {r["trip"]: r for r in ours_m}
    trips = sorted(set(sumo_by) | set(ours_by))

    # Write the full per-trip table first (every trip, arrived or not).
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        if run_meta:
            for k, v in run_meta.items():
                w.writerow([f"# {k}", v])
            w.writerow([f"# generated", time.strftime("%Y-%m-%d %H:%M:%S")])
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

    # MP-1: if fuel is uniformly zero, tripinfo had no emissions data and the
    # live fallback is meaningless -- guard against a fake "100% fuel savings".
    if all((f or 0) == 0 for f in sumo_f) and all((f or 0) == 0 for f in ours_f):
        print("\n[WARNING] All fuel readings are 0 -- the ego vType almost certainly "
              "has no emissions/fuel model, or tripinfo had no <emissions> block. "
              "The FUEL comparison below is NOT meaningful; trust the TIME numbers "
              "and give the ego a vType with an HBEFA fuel model (e.g. set "
              "emissionClass on DEFAULT_VEHTYPE in basic.vType.xml).")

    # MP-2: compute savings and win-counts over the SAME population (drop a trip
    # from both only if its SUMO baseline is non-positive, so the percentages and
    # the win-rates always refer to identical trips).
    valid_t = [(s, o) for s, o in zip(sumo_t, ours_t) if s > 0]
    valid_f = [(s, o) for s, o in zip(sumo_f, ours_f) if s > 0]
    time_sav = [100.0 * (s - o) / s for s, o in valid_t]
    fuel_sav = [100.0 * (s - o) / s for s, o in valid_f]
    time_wins = sum(1 for s, o in valid_t if o < s)
    fuel_wins = sum(1 for s, o in valid_f if o < s)
    n_time, n_fuel = len(valid_t), len(valid_f)

    def block(name, sumo_vals, ours_vals, sav):
        print(f"\n-- {name} --")
        print(f"  SUMO   mean / median : {_mean(sumo_vals):10.1f} / {_median(sumo_vals):10.1f}")
        print(f"  OURS   mean / median : {_mean(ours_vals):10.1f} / {_median(ours_vals):10.1f}")
        print(f"  total  SUMO  / OURS  : {sum(sumo_vals):10.1f} / {sum(ours_vals):10.1f}")
        print(f"  mean per-trip saving : {_mean(sav):+.2f}%   (positive = ours better)")

    block("TIME (s)", sumo_t, ours_t, time_sav)
    print(f"  ours faster on        : {time_wins}/{n_time} trips "
          f"({100.0*time_wins/n_time:.0f}%)" if n_time else "  (no valid time trips)")

    block("FUEL (mg)", sumo_f, ours_f, fuel_sav)
    print(f"  ours used less fuel on: {fuel_wins}/{n_fuel} trips "
          f"({100.0*fuel_wins/n_fuel:.0f}%)" if n_fuel else "  (no valid fuel trips)")

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


def compare_pooled(sumo_lists, ours_lists, run_meta):
    """
    Pools multi-seed results, checks the pre-injection fairness fingerprint,
    and runs paired statistics.
    sumo_lists: list of live_results lists from 'sumo' arms
    ours_lists: list of live_results lists from 'ours' arms
    """
    sumo_all = [r for run in sumo_lists for r in run]
    ours_all = [r for run in ours_lists for r in run]

    # Index by (seed, trip)
    sumo_by = {(r["seed"], r["trip"]): r for r in sumo_all}
    ours_by = {(r["seed"], r["trip"]): r for r in ours_all}
    keys = sorted(set(sumo_by) | set(ours_by))

    stamp = time.strftime("%Y%m%d_%H%M%S")
    csv_path = f"comparison_pooled_{stamp}.csv"

    # Fairness Check
    print("\n" + "=" * 64)
    print("FAIRNESS FINGERPRINT CHECK")
    print("=" * 64)
    mismatches = 0
    for k in keys:
        s, o = sumo_by.get(k), ours_by.get(k)
        if s and o and (s.get("pre_inject_vcount"), s.get("pre_inject_possum")) != \
                       (o.get("pre_inject_vcount"), o.get("pre_inject_possum")):
            print(f"  [Mismatch] seed={k[0]} trip={k[1]}: "
                  f"SUMO={s.get('pre_inject_vcount')}/{s.get('pre_inject_possum')} vs "
                  f"OURS={o.get('pre_inject_vcount')}/{o.get('pre_inject_possum')}")
            mismatches += 1
    if mismatches == 0 and len(keys) > 0:
        print("  PASS: All pre-injection states were byte-identical.")
    else:
        print(f"  FAIL: {mismatches} state drifts detected! (Checkpoint/loadState failed)")

    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        for k, v in run_meta.items():
            w.writerow([f"# {k}", v])
        w.writerow(["seed", "trip", "origin", "dest",
                    "sumo_arrived", "sumo_time_s", "sumo_fuel_mg",
                    "ours_arrived", "ours_time_s", "ours_fuel_mg", "ours_reroutes"])
        for k in keys:
            s, o = sumo_by.get(k, {}), ours_by.get(k, {})
            w.writerow([
                k[0], k[1], s.get("origin", o.get("origin", "")), s.get("dest", o.get("dest", "")),
                s.get("arrived"), s.get("duration_s"), s.get("fuel_mg"),
                o.get("arrived"), o.get("duration_s"), o.get("fuel_mg"), o.get("reroutes")
            ])

    paired = [ (k, sumo_by[k], ours_by[k]) for k in keys 
               if k in sumo_by and k in ours_by and sumo_by[k]["arrived"] and ours_by[k]["arrived"] ]

    print("\n" + "=" * 64)
    print(f"POOLED RESULTS ({len(run_meta['seeds'])} seeds)")
    print("=" * 64)
    print(f"Comparable pairs (both arrived): {len(paired)} / {len(keys)}")
    if not paired:
        return

    sumo_t = [s["duration_s"] for _, s, _ in paired]
    ours_t = [o["duration_s"] for _, _, o in paired]
    sumo_f = [s["fuel_mg"]    for _, s, _ in paired]
    ours_f = [o["fuel_mg"]    for _, _, o in paired]

    valid_t = [(s, o) for s, o in zip(sumo_t, ours_t) if s > 0]
    valid_f = [(s, o) for s, o in zip(sumo_f, ours_f) if s > 0]
    
    if not valid_t and not valid_f:
        print("No valid time/fuel values > 0 to compare.")
        return

    time_sav = [100.0 * (s - o) / s for s, o in valid_t]
    fuel_sav = [100.0 * (s - o) / s for s, o in valid_f]

    def block(name, sumo_vals, ours_vals, sav):
        n_wins = sum(1 for s, o in zip(sumo_vals, ours_vals) if o < s)
        print(f"\n-- {name} --")
        print(f"  SUMO mean/med : {_mean(sumo_vals):8.1f} / {_median(sumo_vals):8.1f}")
        print(f"  OURS mean/med : {_mean(ours_vals):8.1f} / {_median(ours_vals):8.1f}")
        print(f"  Mean savings  : {_mean(sav):+.2f}%")
        print(f"  Ours won      : {n_wins}/{len(sav)} trips ({100.0*n_wins/len(sav) if sav else 0:.0f}%)")

    if valid_t: block("TIME (s)", [s for s, _ in valid_t], [o for _, o in valid_t], time_sav)
    if valid_f: block("FUEL (mg)", [s for s, _ in valid_f], [o for _, o in valid_f], fuel_sav)

    try:
        from scipy import stats
        print("\n-- paired t-test --")
        if valid_t:
            t_t, t_p = stats.ttest_rel([o for _, o in valid_t], [s for s, _ in valid_t])
            print(f"  time: t={t_t:+.3f} p={t_p:.4g}")
        if valid_f:
            f_t, f_p = stats.ttest_rel([o for _, o in valid_f], [s for s, _ in valid_f])
            print(f"  fuel: t={f_t:+.3f} p={f_p:.4g}")
    except ImportError:
        pass

    print(f"\nDetailed CSV written to {csv_path}")


# ---------------------------------------------------------------------------
# 5. Parallel execution: run the two scenarios at the same time
# ---------------------------------------------------------------------------
# WHY MULTIPROCESSING (and not threads / a single labelled-TraCI process):
#   * simulate.py / rsuController.py / this file all use the GLOBAL `traci`
#     module. A separate PROCESS gives each scenario its own `traci` connection
#     and its own SUMO instance with no code changes to those modules.
#   * The per-step Python work (RSUManager.step over all edges + Dijkstra) is
#     real CPU and GIL-bound, so threads would serialise it. Separate processes
#     run on separate cores -> genuine 2x speed-up on the Python side too.
#   * Each scenario already launches an INDEPENDENT SUMO with the same seed and
#     the same OD list, so running them side-by-side instead of back-to-back is
#     bit-identical to the sequential run -- only the wall-clock changes.
#
# The "spawn" start method is used for portability (clean interpreter per child;
# safe with traci/SUMO on Linux, macOS and Windows alike). Each child re-imports
# this module -- which re-runs the SUMO_HOME bootstrap, inherited from the parent
# environment -- and runs exactly one scenario, i.e. one traci.start/traci.close.


def _run_scenario_capture(mode, od_list, traffic_seed, tag, warmup_steps,
                          log_every, scale, teleport, checkpoints=None, rewarm_steps=REWARM_STEPS):
    """
    Process-pool worker. Runs ONE scenario and returns a picklable dict.

    stdout from the scenario (which is chatty via verbose=True) is buffered so
    the two parallel runs don't interleave their logs into an unreadable mess;
    the parent replays each buffer in full as the scenario finishes. The live
    per-trip CSV (progress_<tag>.csv) is still written to disk in real time.

    Must stay a module-level function so it is importable by name under the
    "spawn" start method.
    """
    t0 = time.time()
    buf = io.StringIO()
    error = None
    live, ti = [], {}
    try:
        with redirect_stdout(buf):
            live, ti = run_scenario(mode, od_list, traffic_seed, tag,
                                    warmup_steps, log_every, scale, teleport,
                                    checkpoints=checkpoints, rewarm_steps=rewarm_steps)
    except (KeyboardInterrupt, SystemExit):
        # LP-4: a Ctrl+C in the parent reaches workers as SIGINT. Do NOT swallow
        # it as a "result" -- let it propagate so the run actually aborts.
        raise
    except Exception as e:  # noqa: BLE001 - report ordinary failures as a result
        error = repr(e)
    return {
        "tag": tag,
        "mode": mode,
        "log": buf.getvalue(),
        "error": error,
        "live": live,
        "tripinfo": ti,
        "wall_s": time.time() - t0,
    }


def run_parallel(od_list, traffic_seed, jobs, warmup_steps=WARMUP_STEPS,
                 log_every=PROGRESS_LOG_EVERY, scale=DEFAULT_SCALE,
                 teleport=DEFAULT_TELEPORT, checkpoints=None, rewarm_steps=REWARM_STEPS):
    """
    Launch every (mode, tag) in `jobs` concurrently, one process each.
    Returns {tag: (live_results, tripinfo)}.
    """
    print(f"\nLaunching {len(jobs)} scenarios IN PARALLEL "
          f"(one SUMO instance + one core each).")
    print("Each scenario's log is buffered and printed in full as it finishes, "
          "so the two runs don't interleave.")
    print("Live per-trip progress is written to progress_<tag>.csv "
          "(tail -f to watch it fill up).\n")

    results = {}
    wall0 = time.time()
    ctx = multiprocessing.get_context("spawn")
    ex = concurrent.futures.ProcessPoolExecutor(max_workers=len(jobs), mp_context=ctx)
    try:
        fut_to_tag = {
            ex.submit(_run_scenario_capture, mode, od_list, traffic_seed, tag,
                      warmup_steps, log_every, scale, teleport,
                      checkpoints, rewarm_steps): tag
            for mode, tag in jobs
        }
        # HP-1: bound the wait so a deadlocked SUMO can't hang the parent forever.
        try:
            done_iter = concurrent.futures.as_completed(fut_to_tag,
                                                        timeout=WORKER_TIMEOUT_S)
            for fut in done_iter:
                tag = fut_to_tag[fut]
                try:
                    out = fut.result()
                except Exception as e:  # pragma: no cover - pool-level failure
                    print(f"\n[FATAL] scenario '{tag}' crashed in its worker: {e!r}")
                    results[tag] = ([], {})
                    continue

                print("\n" + "#" * 64)
                print(f"# scenario '{out['tag']}' (ego_routing={out['mode']}) "
                      f"finished in {out['wall_s']:.1f}s")
                print("#" * 64)
                print(out["log"], end="" if out["log"].endswith("\n") else "\n")
                if out["error"]:
                    print(f"[ERROR] scenario '{out['tag']}' raised: {out['error']}")
                results[out["tag"]] = (out["live"], out["tripinfo"])
        except concurrent.futures.TimeoutError:
            unfinished = [t for t in fut_to_tag.values() if t not in results]
            print(f"\n[TIMEOUT] {WORKER_TIMEOUT_S}s elapsed; scenarios still "
                  f"running: {unfinished}. Cancelling and aborting.")
            for t in unfinished:
                results[t] = ([], {})
    except KeyboardInterrupt:
        print("\n[ABORT] interrupted by user; shutting down workers.")
        ex.shutdown(wait=False, cancel_futures=True)
        raise
    finally:
        ex.shutdown(wait=False, cancel_futures=True)

    print(f"\nBoth scenarios done; parallel wall-clock: {time.time() - wall0:.1f}s "
          f"(vs the sum of the two individual times if run sequentially).")
    return results


# ---------------------------------------------------------------------------

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
    print(f"\n=======================================================")
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=100, help="number of OD pairs")
    ap.add_argument("--od-seed", type=int, default=42, help="seed for OD generation")
    ap.add_argument("--traffic-seed", type=int, default=1, help="SUMO traffic seed (same for both runs)")
    ap.add_argument("--legacy", action="store_true",
                    help="run the legacy un-checkpointed run_od_campaign (back-to-back runs)")
    ap.add_argument("--seeds", type=str, default="1",
                    help="comma-separated SUMO traffic seeds (used in new checkpoint mode)")
    ap.add_argument("--sequential", action="store_true",
                    help="run the two scenarios one after another (old behaviour) "
                         "instead of in parallel -- useful for low-RAM machines or debugging")
    ap.add_argument("--log-every", type=int, default=PROGRESS_LOG_EVERY,
                    help="flush the live per-trip log (progress_<tag>.csv) every N trips "
                         "(default 1 = every trip)")
    ap.add_argument("--scale", type=float, default=DEFAULT_SCALE,
                    help="background demand multiplier (HP-2). >1 can saturate MoST; "
                         "lower it if you see repeated trip TIMEOUTs")
    ap.add_argument("--teleport", type=int, default=DEFAULT_TELEPORT,
                    help="seconds a vehicle may be stuck before SUMO teleports it "
                         "(HP-2). -1 = never (honest timing but can deadlock at high "
                         "--scale); 300 is a safe finite default")
    ap.add_argument("--paired", action="store_true", help="enable same-seed paired-run fuel experiment")
    ap.add_argument("--baseline", choices=["sumo", "ablation", "both"], default="both", help="baseline to compare ours against")
    ap.add_argument("--depart-start", type=float, default=21600, help="sim time of first ego departure")
    ap.add_argument("--depart-spacing", type=float, default=120, help="steps between ego departures")
    ap.add_argument("--reroute-interval", type=int, default=REROUTE_INTERVAL, help="reroute cadence")
    ap.add_argument("--use-hysteresis", action="store_true", help="re-enable ours hysteresis")
    args = ap.parse_args()

    od_list = generate_od_pairs(NET_FILE, n=args.n, seed=args.od_seed)
    if not od_list:
        sys.exit("No OD pairs generated; aborting.")

    # Same seed + same OD list for both arms, so the only systematic difference
    # is who routes the ego. Parallel and sequential give identical results.
    jobs = [("sumo", "sumo"), ("ours", "ours")]

    if args.paired:
        print("\n[Paired Mode] Running fixed-departure continuous fuel campaign.")
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
            print(f"\n--- SEED {seed} ---")
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
                print(f"\n# scenario '{out['tag']}' finished in {out['wall_s']:.1f}s")
                if out["error"]: print(f"[ERROR] {out['error']}")
                all_res[arm].append(out["results"])
            ex.shutdown(wait=False, cancel_futures=True)
            
        if "sumo" in all_res:
            analyze_paired_results(all_res["ours"], all_res["sumo"], "sumo", run_meta)
        if "ablation" in all_res:
            analyze_paired_results(all_res["ours"], all_res["ablation"], "ablation", run_meta)
            
        return

    if args.legacy:
        print("\n[Legacy Mode] Running un-checkpointed continuous back-to-back campaign.")
        run_meta = {
            "n": args.n, "od_seed": args.od_seed, "traffic_seed": args.traffic_seed,
            "scale": args.scale, "teleport": args.teleport,
            "alpha": ALPHA, "beta": BETA, "gamma": GAMMA,
            "reroute_interval": REROUTE_INTERVAL, "warmup_steps": WARMUP_STEPS,
            "per_trip_timeout": PER_TRIP_TIMEOUT, "ego_type": EGO_TYPE,
            "od_pairs_built": len(od_list),
        }
        if args.sequential:
            print("\nRunning scenarios SEQUENTIALLY (--sequential).")
            results = {}
            for mode, tag in jobs:
                live, ti = run_scenario(mode, od_list, args.traffic_seed, tag=tag,
                                        log_every=args.log_every,
                                        scale=args.scale, teleport=args.teleport)
                results[tag] = (live, ti)
        else:
            results = run_parallel(od_list, args.traffic_seed, jobs,
                                   log_every=args.log_every,
                                   scale=args.scale, teleport=args.teleport)

        sumo_live, sumo_ti = results["sumo"]
        ours_live, ours_ti = results["ours"]

        sumo_m = merge(sumo_live, sumo_ti)
        ours_m = merge(ours_live, ours_ti)
        compare(sumo_m, ours_m, run_meta=run_meta)
    
    else:
        # NEW Checkpointed Multi-Seed Sweep
        seeds = [int(x.strip()) for x in args.seeds.split(",")]
        run_meta = {
            "n": args.n, "od_seed": args.od_seed, "seeds": seeds,
            "scale": args.scale, "teleport": args.teleport,
            "alpha": ALPHA, "beta": BETA, "gamma": GAMMA,
            "ckpt_spacing": CKPT_SPACING, "rewarm_steps": REWARM_STEPS,
        }

        sumo_all, ours_all = [], []
        for seed in seeds:
            print(f"\n==================================================================")
            print(f" STARTING SEED SWEEP: {seed}")
            print(f"==================================================================")
            checkpoints = build_checkpoints(len(od_list), seed, args.scale, args.teleport)
            
            if args.sequential:
                print("\nRunning arms SEQUENTIALLY for this seed (--sequential).")
                for mode, tag in jobs:
                    live, _ = run_scenario(mode, od_list, seed, tag=tag,
                                           log_every=args.log_every, scale=args.scale,
                                           teleport=args.teleport, checkpoints=checkpoints)
                    if tag == "sumo": sumo_all.append(live)
                    else: ours_all.append(live)
            else:
                results = run_parallel(
                    od_list, seed, jobs, checkpoints=checkpoints,
                    scale=args.scale, teleport=args.teleport
                )
                sumo_all.append(results.get("sumo", [])[0] if results.get("sumo") else [])
                ours_all.append(results.get("ours", [])[0] if results.get("ours") else [])

        compare_pooled(sumo_all, ours_all, run_meta)


if __name__ == "__main__":
    # Required for the "spawn" start method when this module is the entry point.
    multiprocessing.freeze_support()
    main() 
