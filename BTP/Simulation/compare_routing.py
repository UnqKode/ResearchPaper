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
PER_TRIP_TIMEOUT = 3000              # steps before a trip is declared "did not arrive"
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


def run_scenario(mode, od_list, traffic_seed, tag, warmup_steps=WARMUP_STEPS,
                 log_every=PROGRESS_LOG_EVERY, scale=DEFAULT_SCALE,
                 teleport=DEFAULT_TELEPORT):
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
    horizon_s = int((warmup_steps + len(od_list) * PER_TRIP_TIMEOUT) * 0.25) + 20000

    port = _free_port()
    sumo_cmd = [
        SUMO_BIN, "-c", CONFIG_FILE,
        "--remote-port", str(port),           # CR-1: explicit distinct port per worker
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
        "--device.rerouting.probability", "1",
        "--device.rerouting.period", str(REROUTE_INTERVAL),
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
                          log_every, scale, teleport):
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
                                    warmup_steps, log_every, scale, teleport)
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
                 teleport=DEFAULT_TELEPORT):
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
                      warmup_steps, log_every, scale, teleport): tag
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
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=100, help="number of OD pairs")
    ap.add_argument("--od-seed", type=int, default=42, help="seed for OD generation")
    ap.add_argument("--traffic-seed", type=int, default=1, help="SUMO traffic seed (same for both runs)")
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
    args = ap.parse_args()

    od_list = generate_od_pairs(NET_FILE, n=args.n, seed=args.od_seed)
    if not od_list:
        sys.exit("No OD pairs generated; aborting.")

    # Same seed + same OD list for both arms, so the only systematic difference
    # is who routes the ego. Parallel and sequential give identical results.
    jobs = [("sumo", "sumo"), ("ours", "ours")]

    # LP-3: record exactly what produced this run so the CSV is reproducible.
    run_meta = {
        "n": args.n, "od_seed": args.od_seed, "traffic_seed": args.traffic_seed,
        "scale": args.scale, "teleport": args.teleport,
        "alpha": ALPHA, "beta": BETA, "gamma": GAMMA,
        "reroute_interval": REROUTE_INTERVAL, "warmup_steps": WARMUP_STEPS,
        "per_trip_timeout": PER_TRIP_TIMEOUT,
        "ego_type": EGO_TYPE,
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


if __name__ == "__main__":
    # Required for the "spawn" start method when this module is the entry point.
    multiprocessing.freeze_support()
    main()