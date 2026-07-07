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
EGO_TYPE     = "ego_petrol"           # uses HBEFA3/PC_G_EU6 emissionClass and has.emissions.device=true
SUMO_BIN     = "sumo"                 # headless; use "sumo-gui" only to eyeball one run

REROUTE_INTERVAL = 30
ALPHA, BETA, GAMMA = 1.0, 8.0, 10.0
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
WORKER_TIMEOUT_S = 86400             # HP-1: hard wall-clock cap per parallel scenario
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

# ---------------------------------------------------------------------------
# Phase 2: Dynamic Degradation Methods
# ---------------------------------------------------------------------------
def select_degraded_edges(net_file, k, traffic_seed, scale, teleport, depart_start):
    """
    Auto-select k edges by running a headless calibration run up to depart_start-300.
    Selects the k edges with the highest vehicle counts that are >= 50m and >= 8m/s.
    """
    print(f"Running headless calibration to select {k} degraded edges...")
    import traci
    from Simulation.simulate import RSUManager
    
    port = _free_port()
    sumo_cmd = [
        SUMO_BIN, "-c", CONFIG_FILE,
        "--seed", str(traffic_seed),
        "--scale", str(scale),
        "--no-step-log", "true",
        "--no-warnings", "true",
        "--time-to-teleport", str(teleport),
    ]
    net = sumolib.net.readNet(net_file)
    
    traci.start(sumo_cmd, port=port)
    from Simulation.simulate import Simulation
    sim = Simulation(net_file)
    rsu_manager = sim.rsu_manager
    # subscribe_edges() must be called after traci.start() so SUMO can register
    # the subscriptions; without this the RSU receives no data and all edges
    # appear identical (vehicle_count=0), causing arbitrary edge selection.
    rsu_manager.subscribe_edges()

    end_time = depart_start - 300
    while traci.simulation.getTime() < end_time:
        traci.simulationStep()
        rsu_manager.step()

    edges = traci.edge.getIDList()
    edge_counts = []
    for e in edges:
        if not e.startswith(":"):
            stats = rsu_manager.get_edge_stats(e)
            if stats:
                try:
                    sumo_edge = net.getEdge(e)
                    if sumo_edge.getLength() >= 50.0 and sumo_edge.getSpeed() >= 8.0:
                        edge_counts.append((e, stats.get("vehicle_count", 0)))
                except:
                    pass
                    
    traci.close()
    
    edge_counts.sort(key=lambda x: x[1], reverse=True)
    # Simple connectivity check: assume top edges don't completely disconnect MoST
    # A full connectivity check is expensive, but MoST is highly redundant.
    selected = [x[0] for x in edge_counts[:k]]
    print(f"Selected degraded edges: {selected}")
    return selected

def _bypass_cost(net, from_edge, to_edge, blocked_ids, vclass="passenger"):
    """Travel-time cost of the shortest path from_edge→to_edge avoiding blocked_ids.

    Uses an edge-based Dijkstra (travel time = length / speed_limit).
    The origin and destination edges are never blocked even if they appear in
    blocked_ids, so the OD pair always has a well-defined start/end.
    Returns float('inf') if no such path exists.
    """
    import heapq
    INF = float("inf")
    blocked = blocked_ids - {from_edge.getID(), to_edge.getID()}

    start_t = from_edge.getLength() / max(from_edge.getSpeed(), 0.1)
    dist = {from_edge.getID(): start_t}
    heap = [(start_t, from_edge.getID())]

    while heap:
        d, eid = heapq.heappop(heap)
        if d > dist.get(eid, INF):
            continue
        if eid == to_edge.getID():
            return d
        try:
            edge = net.getEdge(eid)
        except Exception:
            continue
        for succ in edge.getToNode().getOutgoing():
            sid = succ.getID()
            if sid in blocked or not succ.allows(vclass):
                continue
            nd = d + succ.getLength() / max(succ.getSpeed(), 0.1)
            if nd < dist.get(sid, INF):
                dist[sid] = nd
                heapq.heappush(heap, (nd, sid))
    return INF


def _ttime_route(net, from_edge, to_edge, blocked_ids, vclass="passenger"):
    """Travel-time shortest path avoiding blocked_ids. Returns (path_edge_id_list, cost_s).

    Like _bypass_cost but also returns the path so callers can inspect which
    edges it crosses. Returns ([], inf) if no path exists.
    """
    import heapq
    INF = float("inf")
    blocked = blocked_ids - {from_edge.getID(), to_edge.getID()}
    start_t = from_edge.getLength() / max(from_edge.getSpeed(), 0.1)
    dist = {from_edge.getID(): start_t}
    prev = {from_edge.getID(): None}
    heap = [(start_t, from_edge.getID())]
    while heap:
        d, eid = heapq.heappop(heap)
        if d > dist.get(eid, INF):
            continue
        if eid == to_edge.getID():
            path = []
            cur = eid
            while cur is not None:
                path.append(cur)
                cur = prev.get(cur)
            return list(reversed(path)), d
        try:
            edge = net.getEdge(eid)
        except Exception:
            continue
        for succ in edge.getToNode().getOutgoing():
            sid = succ.getID()
            if sid in blocked or not succ.allows(vclass):
                continue
            nd = d + succ.getLength() / max(succ.getSpeed(), 0.1)
            if nd < dist.get(sid, INF):
                dist[sid] = nd
                prev[sid] = eid
                heapq.heappush(heap, (nd, sid))
    return [], INF


def select_targeted_od_pairs(net_file, degraded_edges, n, seed, vclass="passenger",
                              min_len=50.0, max_detour_factor=1.25, max_trip_time=None):
    """
    Select OD pairs satisfying TWO conditions:
      1. The travel-time-optimal path crosses at least one degraded edge.
      2. A bypass (path avoiding all degraded edges) exists and costs at most
         max_detour_factor × the normal travel-time cost.

    Both costs are in seconds so the detour ratio is exact. Condition 2 ensures
    the routing penalty can actually tip the balance: if the only detour is 3×
    longer the router will never avoid the degraded edge regardless of penalty.

    max_trip_time: if set, only accept pairs whose free-flow travel time <= this
    value (seconds). Shorter trips make the degraded corridor a larger fraction of
    the total trip, so the fuel penalty has more leverage over bypass congestion.
    """
    desc = f"with viable bypass (detour <={max_detour_factor}x)"
    if max_trip_time:
        desc += f", trip <={max_trip_time}s free-flow"
    print(f"Building {n} targeted OD pairs crossing {degraded_edges} {desc}...")
    net = sumolib.net.readNet(net_file)
    candidates = [e for e in net.getEdges()
                  if (not e.isSpecial()) and e.allows(vclass) and e.getLength() >= min_len]

    blocked_ids = set(degraded_edges)
    rng = random.Random(seed)
    pairs = []
    seen = set()
    attempts = 0
    # Short-trip filter rejects most pairs; need many more attempts to find n valid ones
    max_attempts = n * (10000 if max_trip_time else 1000)

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

        if a.getID() in blocked_ids or b.getID() in blocked_ids:
            continue  # ego can't route around its own start/end edge

        path_ids, cost = _ttime_route(net, a, b, set(), vclass)
        if not path_ids or cost >= float("inf"):
            continue

        if max_trip_time and cost > max_trip_time:
            continue   # trip too long; corridor fraction too small for F penalty to dominate

        if not any(de in path_ids for de in degraded_edges):
            continue   # travel-time-optimal path doesn't cross any degraded edge

        bypass = _bypass_cost(net, a, b, blocked_ids, vclass)
        if bypass > max_detour_factor * cost:
            continue   # bypass unreachable or too expensive; router can't benefit

        corridor_s = sum(net.getEdge(de).getLength() / max(net.getEdge(de).getSpeed(), 0.1)
                         for de in degraded_edges if de in path_ids)
        pairs.append(key)
        print(f"  OD pair {len(pairs)}: trip={cost:.1f}s free-flow, "
              f"corridor={corridor_s:.1f}s ({100*corridor_s/cost:.0f}%), "
              f"bypass={bypass:.1f}s ({100*bypass/cost:.0f}%)")

    n_found = len(pairs)
    if n_found < n:
        print(f"  WARNING: only found {n_found} targeted pairs with viable bypass "
              f"after {attempts} attempts.")
    else:
        print(f"  Got {n_found} targeted pairs with viable bypass in {attempts} attempts.")
    return pairs



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
        "--device.emissions.probability", "1",
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
        "--device.emissions.probability", "1",
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

# ---------------------------------------------------------------------------
# Change 3: Non-bottleneck grade corridor selection
# ---------------------------------------------------------------------------

def _filter_nonbottleneck_candidates(edge_records, p40_occ, speed_ratio_min,
                                      min_len, min_traversals):
    """Pure function: filter edge_records to non-bottleneck free-flowing candidates.

    edge_records: list of dicts with keys:
        eid, avg_occ (normalised [0,1]), avg_spd, spd_lim, length, traversals
    p40_occ: network 40th-percentile occupancy (normalised [0,1]); upper bound
    speed_ratio_min: lower bound on avg_spd / spd_lim (default 0.85); free-flow gate
    min_len: minimum edge length in metres (default 100.0)
    min_traversals: minimum expected traversal count during probe (default 30)

    Returns list of eids that pass all four filters:
    1. Non-bottleneck: occ_floor <= avg_occ <= p40_occ  (not too empty, not congested)
    2. Free-flowing:  avg_spd / spd_lim >= speed_ratio_min
    3. Long enough:   length >= min_len
    4. Observed:      traversals >= min_traversals
    """
    OCC_FLOOR = 0.005   # exclude totally unused edges (< 0.5% occupancy)
    out = []
    for r in edge_records:
        if r["avg_occ"] < OCC_FLOOR or r["avg_occ"] > p40_occ:
            continue
        if r["spd_lim"] <= 0:
            continue
        ratio = r["avg_spd"] / r["spd_lim"]
        if ratio < speed_ratio_min:
            continue
        if r["length"] < min_len:
            continue
        if r["traversals"] < min_traversals:
            continue
        out.append(r["eid"])
    return out


def select_nonbottleneck_grade_edges(net_file, config_file=None, k=2,
                                     od_coverage_min=0.20,
                                     od_list=None,
                                     probe_seed=42,
                                     sumo_bin="sumo",
                                     scale=1.0, teleport=300,
                                     min_len=100.0,
                                     spd_lim_min=7.0,
                                     spd_lim_max=20.0,
                                     probe_begin=None):
    """Return k edge-ids suitable for grade degradation using sumolib only (no probe).

    Selection criteria (all evaluated offline via sumolib — no SUMO/TraCI probe):

    1. Urban speed limit: spd_lim_min <= speed_limit <= spd_lim_max (default 7–20 m/s).
       This encodes "free-flowing at baseline" structurally: urban arterials at
       their posted limit are not chronically congested.  Highways (>20 m/s) and
       side streets (<7 m/s) are excluded.  Degrading these edges under the
       HBEFA grade emission class raises fuel WITHOUT materially raising travel time.
    2. Long enough: edge length >= min_len metres (default 100 m).
    3. OD-relevant: appears on the sumolib shortest path of >= od_coverage_min
       fraction of the OD sample (default 0.20), ensuring ego trips cross it.
       If od_list is None or empty a throwaway 50-pair sample is generated.

    config_file and probe-related parameters (sumo_bin, scale, teleport,
    probe_begin) are accepted but unused — kept for call-site compatibility.

    Returns a list of k edge IDs, or raises RuntimeError if no eligible set found.
    """
    # --- Load net (sumolib only — no SUMO process needed) ---
    net = sumolib.net.readNet(net_file, withInternal=False)
    all_edges = [e for e in net.getEdges() if not e.getID().startswith(":")]

    # --- Filter 1 + 2: speed limit range and length ---
    candidates = []
    for e in all_edges:
        spd_lim = e.getSpeed()
        if spd_lim < spd_lim_min or spd_lim > spd_lim_max:
            continue
        if e.getLength() < min_len:
            continue
        candidates.append(e.getID())

    if not candidates:
        raise RuntimeError("select_nonbottleneck_grade_edges: no edges passed "
                           "speed-limit/length filter")

    # --- Filter 5: OD coverage via sumolib (no TraCI, no Simulation instance) ---
    # If no od_list supplied, generate a throwaway 50-pair sample so coverage
    # is never silently skipped (Fix 3 ordering note: caller should pass its
    # campaign od_list when available; see IMPLEMENTATION_NOTES.md).
    if not od_list:
        od_list = generate_od_pairs(net_file, n=50, seed=probe_seed)

    cand_set = set(candidates)
    cov = {e: 0 for e in candidates}
    n_od = 0
    for orig_id, dest_id in od_list:
        try:
            o = net.getEdge(orig_id)
            d = net.getEdge(dest_id)
            path, _cost = net.getShortestPath(o, d)
            if not path:
                continue
            n_od += 1
            for pe in path:
                pid = pe.getID()
                if pid in cand_set:
                    cov[pid] += 1
        except Exception:
            continue

    if n_od > 0:
        candidates = [e for e in candidates
                      if cov[e] / n_od >= od_coverage_min]

    if not candidates:
        raise RuntimeError("select_nonbottleneck_grade_edges: no edges passed "
                           "OD coverage filter")

    # Return k edges; prefer shorter IDs (tends to pick canonical segment names)
    candidates.sort(key=lambda e: (len(e), e))
    return candidates[:k]


def _arm_params(arm, args):
    """Return (alpha, beta, gamma, cost_mode) for an arm name.

    Change 3: supports the extended --arms flag with ours-fuel / ours-augtime /
    ablation arm names in addition to the legacy ours / sumo / ablation.
    Two-arm invocations (legacy) remain byte-compatible: when --arms is not set
    the caller still uses arms_to_run built from --baseline, and arm=="ours" falls
    through to the args.cost_mode branch.
    """
    if arm == "ablation":
        return 0.0, 0.0, 0.0, "augtime"
    elif arm == "sumo":
        return 0.0, 0.0, 0.0, "augtime"
    elif arm == "ours-fuel":
        return ALPHA, BETA, GAMMA, "fuel"
    elif arm == "ours-augtime":
        return ALPHA, BETA, GAMMA, "augtime"
    else:  # "ours" — use the CLI cost-mode
        return ALPHA, BETA, GAMMA, args.cost_mode


def run_paired_scenario(arm_policy, alpha, beta, gamma, od_list, traffic_seed, tag,
                        depart_start, depart_spacing, use_hysteresis,
                        scale, teleport, road_condition_manager=None, debug_cfs=False,
                        diag_stamp=None, theta_fuel=1.0, theta_time=0.10,
                        cost_mode="augtime",
                        fuel_aggregator="median", fuel_sample_max_age_s=600.0,
                        junction_weight=1.0, fuel_hysteresis=0.10):
    out_prefix = f"{tag}."
    tripinfo_path = f"{out_prefix}tripinfo.xml"
    progress_path = f"progress_{tag}.csv"

    try: os.remove(tripinfo_path)
    except OSError: pass
    try: os.remove(progress_path)
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
        "--device.emissions.probability", "1",
        "--device.rerouting.period", str(REROUTE_INTERVAL),
        "--route-steps", "0"
    ]
    print(f"[SUMO_CMD] tag={tag} seed={traffic_seed} cmd={' '.join(sumo_cmd)}")
    sys.stdout.flush()
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
            ego_od_list=od_list,
            debug_cfs=debug_cfs,
            road_condition_manager=road_condition_manager,
            theta_fuel=theta_fuel,
            theta_time=theta_time,
            cost_mode=cost_mode,
            fuel_aggregator=fuel_aggregator,
            fuel_sample_max_age_s=fuel_sample_max_age_s,
            junction_weight=junction_weight,
            fuel_hysteresis=fuel_hysteresis,
        )
        # D3: bind the unbound manager (pickled without traci/calc refs) to live objects.
        if road_condition_manager is not None and road_condition_manager.mode != "none":
            road_condition_manager.bind(traci, sim.calc, sim.rsu_manager)
            print(
                f"[{tag}] RoadConditionManager bound "
                f"(mode={road_condition_manager.mode}, "
                f"edges={road_condition_manager.degraded_edges})"
            )
        live_results = sim.run_fixed_departure_campaign(
            od_list=od_list,
            depart_start=depart_start,
            depart_spacing=depart_spacing,
            per_trip_timeout=PER_TRIP_TIMEOUT,
            ego_policy=arm_policy,
            reroute_interval=REROUTE_INTERVAL,
            use_hysteresis=use_hysteresis,
            progress_log_path=progress_path,
            seed=traffic_seed,
            diag_stamp=diag_stamp
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
            try:
                import xml.etree.ElementTree as ET
                ET.parse(tripinfo_path)
                break
            except ET.ParseError:
                pass
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

    # Fuel-objective routing: report predicted-vs-realized fuel match rate.
    # "match" = predicted within 20% of realized (|pred-real|/real <= 0.20).
    if cost_mode == "fuel" and arm_policy == "ours":
        pairs = [
            (r["predicted_fuel_mg"], r.get("fuel_abs"))
            for r in live_results
            if r.get("arrived") and r.get("predicted_fuel_mg") is not None
            and r.get("fuel_abs") is not None and r["fuel_abs"] > 0
        ]
        if pairs:
            match_pct = 0.20
            n_match = sum(
                1 for pred, real in pairs
                if abs(pred - real) / real <= match_pct
            )
            ratios = [pred / real for pred, real in pairs]
            mean_ratio = sum(ratios) / len(ratios)
            print(
                f"[FUEL_MATCH] predicted/realized: mean_ratio={mean_ratio:.3f} "
                f"match_within_20pct={n_match}/{len(pairs)} "
                f"seed={traffic_seed}"
            )

    # CHECK 1: driven-route predicted vs realized (calibration test).
    # Uses the FINAL traversal-window state (end-of-run), not per-edge at traversal time.
    # ratio ~1.0 -> cost is calibrated; ratio ~0.39 -> systematic under-prediction bug.
    if cost_mode == "fuel" and arm_policy == "ours":
        try:
            _rcm = getattr(sim, 'road_condition_manager', None)
            _degraded = set(getattr(_rcm, 'degraded_edges', []))
            dm_ratios = []
            for rec in live_results:
                driven = rec.get("driven_edges", [])
                if not rec.get("arrived"):
                    # CHECK 2: non-arriving trip diagnosis (same calibration issue or OD/topology?)
                    if rec.get("trip") == 11:
                        print(
                            f"[TRIP11_NOARRIVE] driven {len(driven)} edges before timeout; "
                            f"on_degraded={[e for e in driven if e in _degraded]}"
                        )
                        print(f"  [TRIP11] per-edge cost at end-of-run "
                              f"(final window, mg/traversal):")
                        for e in driven:
                            seg = sim.calc.get_segment_fuel(e)
                            dq  = sim.calc._traversal_fuel.get(e)
                            st  = sim.rsu_manager.get_edge_stats(e)
                            v   = st.get("avg_speed", 0.0) if st else 0.0
                            tag_d = " <DEGRADED>" if e in _degraded else ""
                            print(f"    {e}: seg={seg:.1f}mg "
                                  f"obs={len(dq) if dq else 0} "
                                  f"v_rsu={v:.2f}m/s{tag_d}")
                    continue
                realized = rec.get("fuel_abs")
                if not realized or realized <= 0:
                    continue
                if not driven:
                    continue
                predicted = sum(sim.calc.get_segment_fuel(e) for e in driven)
                ratio = predicted / realized
                dm_ratios.append(ratio)
                print(f"[DRIVEN_MATCH] ego={rec['trip']} "
                      f"predicted={predicted:.0f} realized={realized:.0f} "
                      f"ratio={ratio:.3f} n_edges={len(driven)}")
                if rec.get("trip") == 11:
                    print(f"  [TRIP11] per-edge cost at end-of-run:")
                    for e in driven:
                        seg = sim.calc.get_segment_fuel(e)
                        dq  = sim.calc._traversal_fuel.get(e)
                        st  = sim.rsu_manager.get_edge_stats(e)
                        v   = st.get("avg_speed", 0.0) if st else 0.0
                        tag_d = " <DEGRADED>" if e in _degraded else ""
                        print(f"    {e}: seg={seg:.1f}mg "
                              f"obs={len(dq) if dq else 0} "
                              f"v_rsu={v:.2f}m/s{tag_d}")
            if dm_ratios:
                mean_dm = sum(dm_ratios) / len(dm_ratios)
                n_dm_match = sum(1 for r in dm_ratios if abs(r - 1.0) <= 0.20)
                verdict = "CALIBRATED" if mean_dm >= 0.80 else "BUG:under-predicted"
                print(
                    f"[DRIVEN_MATCH_SUMMARY] n={len(dm_ratios)} "
                    f"mean_ratio={mean_dm:.3f} "
                    f"within_20pct={n_dm_match}/{len(dm_ratios)} "
                    f"seed={traffic_seed} verdict={verdict}"
                )
        except Exception as _dm_e:
            print(f"[DRIVEN_MATCH] failed: {_dm_e}")

    return live_results, sim.calc.cfs_records if sim.calc else [], sim.route_cfs_records

def _run_paired_capture(arm_policy, alpha, beta, gamma, od_list, traffic_seed, tag,
                        depart_start, depart_spacing, use_hysteresis, scale, teleport,
                        road_condition_manager=None, debug_cfs=False, diag_stamp=None,
                        theta_fuel=1.0, theta_time=0.10, cost_mode="augtime",
                        fuel_aggregator="median", fuel_sample_max_age_s=600.0,
                        junction_weight=1.0, fuel_hysteresis=0.10):
    t0 = time.time()
    buf = io.StringIO()
    error = None
    res = []
    cfs = []
    route_cfs = []
    try:
        with redirect_stdout(buf):
            res, cfs, route_cfs = run_paired_scenario(
                arm_policy, alpha, beta, gamma, od_list, traffic_seed, tag,
                depart_start, depart_spacing, use_hysteresis, scale, teleport,
                road_condition_manager=road_condition_manager, debug_cfs=debug_cfs,
                diag_stamp=diag_stamp, theta_fuel=theta_fuel, theta_time=theta_time,
                cost_mode=cost_mode,
                fuel_aggregator=fuel_aggregator,
                fuel_sample_max_age_s=fuel_sample_max_age_s,
                junction_weight=junction_weight,
                fuel_hysteresis=fuel_hysteresis,
            )
    except Exception as e:
        error = repr(e)
        import sys as _sys
        _sys.stderr.write(f"[ARM_CRASH] {tag}: {error}\n")
        import traceback as _tb
        _sys.stderr.write(_tb.format_exc())
        _sys.stderr.flush()
    return {
        "tag": tag, "log": buf.getvalue(), "error": error,
        "results": res, "cfs_records": cfs, "route_cfs_records": route_cfs,
        "wall_s": time.time() - t0
    }

def analyze_paired_results(ours_lists, base_lists, base_name, run_meta,
                           ours_label="ours"):
    """Compare ours_lists against base_lists arm and write per-ego, per-seed, summary.

    Change 3: `ours_label` names the "ours" arm in output files, enabling
    multi-arm runs (ours-fuel, ours-augtime, etc.) to produce distinct files
    without overwriting each other.  Legacy two-arm calls pass no ours_label
    (default "ours") and remain byte-compatible.
    """
    import json
    try: from scipy import stats
    except ImportError: stats = None

    stamp = time.strftime("%Y%m%d_%H%M%S")
    f_ego  = f"paired_per_ego_{stamp}_{ours_label}_vs_{base_name}.csv"
    f_seed = f"paired_per_seed_{stamp}_{ours_label}_vs_{base_name}.csv"
    f_sum  = f"paired_summary_{stamp}_{ours_label}_vs_{base_name}.json"
    
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

        # Report non-arriving trips before filtering — these are excluded from
        # fuel/time means and must be reported separately (timeout contamination fix).
        ours_noarrive = [k for k in seed_keys if k in ours_by and not ours_by[k].get("arrived")]
        base_noarrive = [k for k in seed_keys if k in base_by and not base_by[k].get("arrived")]
        ours_arrive   = [k for k in seed_keys if k in ours_by and ours_by[k].get("arrived")]
        base_arrive   = [k for k in seed_keys if k in base_by and base_by[k].get("arrived")]
        print(f"[ARRIVAL_COUNTS] seed={s}: "
              f"ours={len(ours_arrive)} arrived / {len(ours_noarrive)} did-not-arrive; "
              f"{base_name}={len(base_arrive)} arrived / {len(base_noarrive)} did-not-arrive")
        if ours_noarrive:
            print(f"  ours non-arrivals: trips {sorted(k[1] for k in ours_noarrive)}")
        if base_noarrive:
            print(f"  {base_name} non-arrivals: trips {sorted(k[1] for k in base_noarrive)}")

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

    # ------------------------------------------------------------------
    # D5 — Gate C: avoidance-rate analysis
    # ------------------------------------------------------------------
    degraded_edges = run_meta.get("degraded_edges", [])
    if degraded_edges:
        print(f"\n-- Gate C: avoidance of degraded edges {degraded_edges} --")

        def _avoidance(flat_list, label):
            avoided, crossed, total_arrived = 0, 0, 0
            for r in flat_list:
                if not r.get("arrived"):
                    continue
                total_arrived += 1
                driven = r.get("driven_edges", [])
                if isinstance(driven, str):
                    driven = driven.split("|") if driven else []
                if any(de in driven for de in degraded_edges):
                    crossed += 1
                else:
                    avoided += 1
            rate = avoided / total_arrived * 100 if total_arrived > 0 else 0.0
            print(f"  {label}: avoided={avoided}/{total_arrived} ({rate:.1f}%), "
                  f"crossed={crossed}/{total_arrived}")
            return rate, avoided, crossed, total_arrived

        ours_avoidance_rate, _, _, _ = _avoidance(ours_flat, "ours")
        base_avoidance_rate, _, _, _ = _avoidance(base_flat, base_name)

        summary["gate_c"] = {
            "degraded_edges": degraded_edges,
            "road_condition": run_meta.get("road_condition", "unknown"),
            "avoidance_rate_ours_pct": ours_avoidance_rate,
            f"avoidance_rate_{base_name}_pct": base_avoidance_rate,
        }

        mean_fuel_sav = summary.get("fuel_sav_mean", 0.0)
        if mean_fuel_sav > 0 and ours_avoidance_rate <= base_avoidance_rate:
            print("\n" + "*" * 65)
            print("  SUSPECTED ARTIFACT: fuel saving without avoidance signature")
            print("  ours saved fuel but did NOT avoid degraded edges more than")
            print(f"  {base_name} (ours={ours_avoidance_rate:.1f}% <= "
                  f"{base_name}={base_avoidance_rate:.1f}%).")
            print("  Result may be a statistical artefact — do not claim as proof.")
            print("*" * 65)
            summary["gate_c"]["artifact_flag"] = True
        else:
            summary["gate_c"]["artifact_flag"] = False

    # ------------------------------------------------------------------
    # Change 3 — Gate A′: degraded-edge fuel ≥2×, time <10% on crossed trips
    # ------------------------------------------------------------------
    if degraded_edges:
        crossed_ours_fuel  = []
        crossed_ours_time  = []
        crossed_base_fuel  = []
        crossed_base_time  = []
        for r in ours_flat:
            if not r.get("arrived"): continue
            driven = r.get("driven_edges", [])
            if isinstance(driven, str): driven = driven.split("|") if driven else []
            if any(de in driven for de in degraded_edges):
                if r.get("fuel_abs") is not None:
                    crossed_ours_fuel.append(r["fuel_abs"])
                if r.get("duration") is not None:
                    crossed_ours_time.append(r["duration"])
        for r in base_flat:
            if not r.get("arrived"): continue
            driven = r.get("driven_edges", [])
            if isinstance(driven, str): driven = driven.split("|") if driven else []
            if any(de in driven for de in degraded_edges):
                if r.get("fuel_abs") is not None:
                    crossed_base_fuel.append(r["fuel_abs"])
                if r.get("duration") is not None:
                    crossed_base_time.append(r["duration"])
        if crossed_ours_fuel and crossed_base_fuel and crossed_ours_time and crossed_base_time:
            ratio_fuel = (sum(crossed_ours_fuel)/len(crossed_ours_fuel)) / \
                         (sum(crossed_base_fuel)/len(crossed_base_fuel))
            ratio_time = (sum(crossed_ours_time)/len(crossed_ours_time)) / \
                         (sum(crossed_base_time)/len(crossed_base_time))
            gate_a_prime = (ratio_fuel >= 2.0) and (abs(ratio_time - 1.0) < 0.10)
            summary["gate_a_prime"] = {
                "fuel_ratio_on_degraded": ratio_fuel,
                "time_ratio_on_degraded": ratio_time,
                "passes": gate_a_prime,
            }
            print(f"\n-- Gate A′ (degraded-edge impact) --")
            print(f"  fuel ratio (ours/base on degraded): {ratio_fuel:.2f}x  "
                  f"(need >=2.0)  {'PASS' if ratio_fuel >= 2.0 else 'FAIL'}")
            print(f"  time ratio (ours/base on degraded): {ratio_time:.3f}  "
                  f"(need <1.10)  {'PASS' if abs(ratio_time-1.0)<0.10 else 'FAIL'}")
            print(f"  Gate A′ overall: {'PASS' if gate_a_prime else 'FAIL'}")

    # ------------------------------------------------------------------
    # Change 3 — Gate C′: ours-fuel avoidance > base avoidance
    # ------------------------------------------------------------------
    if degraded_edges and ours_label == "ours-fuel":
        ours_avoidance_rate_c = summary.get("gate_c", {}).get("avoidance_rate_ours_pct", 0.0)
        base_avoidance_rate_c = summary.get("gate_c", {}).get(
            f"avoidance_rate_{base_name}_pct", 0.0)
        gate_c_prime = ours_avoidance_rate_c > base_avoidance_rate_c
        summary["gate_c_prime"] = {
            "ours_fuel_avoidance_pct": ours_avoidance_rate_c,
            f"{base_name}_avoidance_pct": base_avoidance_rate_c,
            "passes": gate_c_prime,
        }
        print(f"\n-- Gate C′ (ours-fuel avoids > {base_name}) --")
        print(f"  ours-fuel={ours_avoidance_rate_c:.1f}%  "
              f"{base_name}={base_avoidance_rate_c:.1f}%  "
              f"{'PASS' if gate_c_prime else 'FAIL'}")

    # ------------------------------------------------------------------
    # Fix 6 — Strengthened fuel-specificity statistics
    # ------------------------------------------------------------------
    # Helper: simple OLS Δfuel% = a + b·Δtime% on a list of (df, dt) pairs.
    # Returns (a, b, se_a, ci_a) or (a, b, None, None) when scipy unavailable.
    def _ols_intercept(pairs_df_dt, stats_mod):
        n   = len(pairs_df_dt)
        if n < 3:
            return None, None, None, None
        sx  = sum(dt for _, dt in pairs_df_dt)
        sy  = sum(df for df, _ in pairs_df_dt)
        sxx = sum(dt**2 for _, dt in pairs_df_dt)
        sxy = sum(df*dt for df, dt in pairs_df_dt)
        denom = n*sxx - sx**2
        if denom == 0:
            return None, None, None, None
        b   = (n*sxy - sx*sy) / denom
        a   = (sy - b*sx) / n
        resids = [df - (a + b*dt) for df, dt in pairs_df_dt]
        rss = sum(r**2 for r in resids)
        se_a = ci_a = None
        if stats_mod and n > 2:
            s2      = rss / (n - 2)
            sxx_bar = sxx - sx**2 / n
            if sxx_bar > 0:
                se_a  = (s2 * (1/n + (sx/n)**2 / sxx_bar)) ** 0.5
                t_c   = stats_mod.t.ppf(0.975, n - 2)
                ci_a  = (a - t_c * se_a, a + t_c * se_a)
        return a, b, se_a, ci_a

    fs_result = {}
    try:
        # --- 1. Per-seed OLS: collect per-ego Δfuel% and Δtime% within each seed ---
        EQUAL_TIME_THRESH = 5.0   # |Δtime%| <= 5% for equal-time subset
        MIN_EGOS_PER_SEED = 5

        per_seed_intercepts = []   # a_s for each valid seed
        eq_time_ego_dfuel_by_seed = {}  # seed → [Δfuel%] for equal-time egos

        for s in seeds:
            seed_keys = [k for k in keys if k[0] == s]
            paired_s = [
                k for k in seed_keys
                if k in ours_by and k in base_by
                and ours_by[k]["arrived"] and base_by[k]["arrived"]
                and ours_by[k].get("fuel_abs") is not None
                and base_by[k].get("fuel_abs") is not None
            ]
            if len(paired_s) < MIN_EGOS_PER_SEED:
                continue
            pairs_ego = []
            eq_fuels  = []
            for k in paired_s:
                o_f = ours_by[k]["fuel_abs"]
                b_f = base_by[k]["fuel_abs"]
                o_t = ours_by[k]["duration"]
                b_t = base_by[k]["duration"]
                if b_f <= 0 or b_t <= 0:
                    continue
                df_pct = 100.0 * (b_f - o_f) / b_f   # positive = ours saves fuel
                dt_pct = 100.0 * (b_t - o_t) / b_t
                pairs_ego.append((df_pct, dt_pct))
                if abs(dt_pct) <= EQUAL_TIME_THRESH:
                    eq_fuels.append(df_pct)
            a_s, b_s, _, _ = _ols_intercept(pairs_ego, stats)
            if a_s is not None:
                per_seed_intercepts.append(a_s)
            if eq_fuels:
                eq_time_ego_dfuel_by_seed[s] = eq_fuels

        # --- 2. Seed-level stats on per-seed intercepts ---
        n_si = len(per_seed_intercepts)
        mean_int = ci_int = w_p_int = None
        if n_si >= 2:
            mean_int = sum(per_seed_intercepts) / n_si
            if stats:
                try:
                    _, t_p_si = stats.ttest_1samp(per_seed_intercepts, 0.0)
                    std_si    = (sum((x - mean_int)**2 for x in per_seed_intercepts)
                                 / (n_si - 1)) ** 0.5
                    t_c       = stats.t.ppf(0.975, n_si - 1)
                    ci_int    = (mean_int - t_c * std_si / n_si**0.5,
                                 mean_int + t_c * std_si / n_si**0.5)
                    try:
                        _, w_p_int = stats.wilcoxon(per_seed_intercepts,
                                                    alternative="two-sided")
                    except Exception:
                        w_p_int = None
                except Exception:
                    pass

        fs_result["per_seed_intercepts_pct"] = per_seed_intercepts
        fs_result["mean_intercept_pct"]      = mean_int
        fs_result["CI_95_intercept_seedlevel"] = ci_int
        fs_result["wilcoxon_p_intercepts"]   = w_p_int

        # --- 3. Equal-time subset report ---
        eq_seed_means = [sum(v)/len(v) for v in eq_time_ego_dfuel_by_seed.values() if v]
        n_eq_egos     = sum(len(v) for v in eq_time_ego_dfuel_by_seed.values())
        eq_t_p        = None
        eq_mean_dfuel = sum(eq_seed_means) / len(eq_seed_means) if eq_seed_means else None
        if stats and len(eq_seed_means) >= 2:
            try:
                _, eq_t_p = stats.ttest_1samp(eq_seed_means, 0.0)
            except Exception:
                pass
        fs_result["equal_time_subset"] = {
            "n_egos": n_eq_egos,
            "n_valid_seeds": len(eq_seed_means),
            "mean_dfuel_pct": eq_mean_dfuel,
            "t_p_seedlevel":  eq_t_p,
        }

        # --- 4. Seed-level OLS (kept as secondary readout) ---
        fuel_pct_sl = [sr["fuel_saving_pct"] for sr in seed_records]
        time_pct_sl = [sr["dur_saving_pct"]  for sr in seed_records]
        a_sl, b_sl, se_sl, ci_sl = _ols_intercept(
            list(zip(fuel_pct_sl, time_pct_sl)), stats)
        fs_result["seedlevel_ols"] = {
            "intercept_pct": a_sl, "slope": b_sl,
            "se_intercept": se_sl, "CI_95_intercept": ci_sl,
        }

        summary["fuel_specificity"] = fs_result

        print(f"\n[FUEL_SPECIFICITY] Per-seed intercept analysis "
              f"(n_valid_seeds={n_si})")
        if mean_int is not None:
            print(f"[FUEL_SPECIFICITY]   mean intercept = {mean_int:.2f}%", end="")
            if ci_int:
                print(f"  95% CI [{ci_int[0]:.2f}%, {ci_int[1]:.2f}%]", end="")
            print()
            if w_p_int is not None:
                print(f"[FUEL_SPECIFICITY]   Wilcoxon p (intercepts) = {w_p_int:.4f}")
        if eq_mean_dfuel is not None:
            print(f"[FUEL_SPECIFICITY] Equal-time subset "
                  f"(|Δtime%|<={EQUAL_TIME_THRESH}): "
                  f"n_egos={n_eq_egos}  mean_Δfuel={eq_mean_dfuel:.2f}%", end="")
            if eq_t_p is not None:
                print(f"  t-test p={eq_t_p:.4f}", end="")
            print()
        if a_sl is not None:
            print(f"[FUEL_SPECIFICITY] Seed-level OLS: "
                  f"Δfuel% = {a_sl:.2f} + {b_sl:.3f}·Δtime%")
    except Exception as _fs_e:
        print(f"[FUEL_SPECIFICITY] failed: {_fs_e}")

    with open(f_sum, "w") as f: json.dump(summary, f, indent=2)
    return f_sum



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
    ap.add_argument("--seed-parallelism", type=int, default=1,
                    help="number of seeds to run concurrently (each seed uses 2 workers; "
                         "total workers = seed-parallelism * 2; cap to available cores)")
    
    # Phase 1 & 2 additions
    ap.add_argument("--debug-cfs", action="store_true", help="enable Phase 1 C/F/S debug sink and summary")
    ap.add_argument("--theta-fuel", type=float, default=1.0,
                    help="Step 2: min fractional fuel saving for bypass to be accepted (e.g. 0.05 = 5%%). "
                         "Default 1.0 disables the rule (pure CFS routing).")
    ap.add_argument("--theta-time", type=float, default=0.10,
                    help="Step 2: max fractional extra time allowed for bypass (e.g. 0.10 = 10%%). "
                         "Only used when --theta-fuel < 1.0.")
    ap.add_argument("--cost-mode", choices=["augtime", "fuel"], default="augtime",
                    help="Routing cost: 'augtime' (default) = CFS augmented travel time; "
                         "'fuel' = minimise per-traversal fuel (mg). Ablation always uses time.")
    ap.add_argument("--scale-sweep", type=str, default="", help="comma-separated scales for Phase 2 congestion sweep")
    ap.add_argument("--depart-sweep", type=str, default="", help="comma-separated depart_starts for Phase 2 sweep")

    # Phase 3 additions
    ap.add_argument("--fuel-aggregator", choices=["wmean", "median", "trimmed"], default="median",
                    help="Change 2A: aggregation for per-traversal fuel window. "
                         "'wmean' = recency-weighted mean (original), 'median' = robust median (default), "
                         "'trimmed' = mean after dropping extreme min/max. "
                         "Use 'wmean' to reproduce pre-Change-2A behaviour.")
    ap.add_argument("--junction-weight", type=float, default=1.0,
                    help="Change 1: multiplier on the junction entry-stop fuel penalty. "
                         "0.0 disables the penalty entirely (pre-Change-1 behaviour). Default 1.0.")
    ap.add_argument("--fuel-hysteresis", type=float, default=0.10,
                    help="Change 2B: minimum fractional fuel-saving required to accept a reroute "
                         "in fuel mode. E.g. 0.10 = only switch if new route is >=10%% cheaper. "
                         "0.0 disables (always accept). Default 0.10.")
    ap.add_argument("--arms", type=str, default="",
                    help="Change 3: comma-separated arm names to run, overrides --baseline. "
                         "Supported: ours, ours-fuel, ours-augtime, ablation, sumo. "
                         "Example: ours-fuel,ours-augtime,ablation. "
                         "Default (empty) falls back to legacy --baseline behaviour.")
    
    
    # Phase 2: Degraded Road args
    ap.add_argument("--road-condition", choices=["none", "rough", "accident", "grade"], default="none", help="degradation mode")
    ap.add_argument("--degraded-edges", type=str, default="",
                    help="comma-separated list of edges, 'auto' (bottleneck), or "
                         "'auto-nonbottleneck' (Change 3: non-bottleneck grade corridor)")
    ap.add_argument("--n-degraded", type=int, default=3, help="k for auto-selection")
    ap.add_argument("--degrade-start", type=float, default=-1, help="time to start degradation (-1 = depart_start - 300)")
    ap.add_argument("--degrade-vlow", type=float, default=5.0, help="v_low for rough mode")
    ap.add_argument("--degrade-vhigh", type=float, default=12.0, help="v_high for rough mode")
    ap.add_argument("--degrade-period", type=float, default=20.0, help="period for rough mode")
    ap.add_argument("--event-duration", type=float, default=600.0, help="duration for accident mode")
    ap.add_argument("--grade-emission-class", type=str, default="HBEFA3/PC_G_EU0",
                    help="heavier emission class applied to vehicles on degraded edges in grade mode")
    ap.add_argument("--targeted-od", action="store_true", help="force OD generation to target degraded edges")
    ap.add_argument("--max-detour-factor", type=float, default=1.25,
                    help="targeted-OD: only include pairs where the bypass path costs "
                         "at most this multiple of the normal travel-time path (default 1.25). "
                         "Must be >1.0.")
    ap.add_argument("--max-od-trip-time", type=float, default=None,
                    help="targeted-OD: only include pairs whose free-flow trip time is at most "
                         "this many seconds (default: no limit). Shorter trips make the degraded "
                         "corridor a larger fraction of the total trip, giving the F penalty "
                         "more leverage over bypass congestion costs.")
    ap.add_argument("--verify-degradation", action="store_true", help="Gate A verification: run headless and check fuel per edge")
    
    args = ap.parse_args()

    
    depart_start = args.depart_start
    if args.degrade_start < 0:
        degrade_start = depart_start - 300
    else:
        degrade_start = args.degrade_start

    degraded_edges = []
    _nonbottleneck_deferred = False   # Fix 3: selector runs after OD generation
    if args.road_condition != "none":
        if args.degraded_edges == "auto":
            degraded_edges = select_degraded_edges(NET_FILE, args.n_degraded, args.traffic_seed, args.scale, args.teleport, depart_start)
        elif args.degraded_edges == "auto-nonbottleneck":
            # Fix 3: defer edge selection until after OD generation so the real
            # campaign OD list can be passed for coverage filtering.  See
            # IMPLEMENTATION_NOTES.md (Review fixes, Fix 3) for ordering rationale.
            _nonbottleneck_deferred = True
        elif args.degraded_edges:
            degraded_edges = [x.strip() for x in args.degraded_edges.split(",") if x.strip()]

    if args.verify_degradation:
        if not degraded_edges:
            sys.exit("Gate A Error: --verify-degradation requires degraded edges to be set.")
        print(f"\n=======================================================")
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
        from Simulation.simulate import Simulation
        sim = Simulation(NET_FILE)
        rsu_manager = sim.rsu_manager
        calc = sim.calc
        rsu_manager.subscribe_edges()
        
        manager = RoadConditionManager(
            degraded_edges, args.road_condition, degrade_start,
            v_low=args.degrade_vlow, v_high=args.degrade_vhigh,
            period_s=args.degrade_period, event_duration=args.event_duration,
            grade_emission_class=args.grade_emission_class
        )
        manager.bind(traci, calc)
        
        # Gate A: measure per-vehicle fuel and speed directly from TraCI each step.
        # Avoids RSU EMA artifacts on sparse-traffic edges.
        # Accumulate (fuel_rate_mg_per_s, speed_m_per_s) for each vehicle on each edge.
        fuel_before = {e: [] for e in degraded_edges}   # list of mg/s rates
        fuel_after  = {e: [] for e in degraded_edges}
        spd_before  = {e: [] for e in degraded_edges}   # list of m/s speeds
        spd_after   = {e: [] for e in degraded_edges}
        # Diagnostics: total vehicle-steps seen, and how many had fuel > 0
        _diag_total  = {e: 0 for e in degraded_edges}
        _diag_nonzero = {e: 0 for e in degraded_edges}
        _diag_printed = False

        end_time = depart_start + 1800  # 30 min post-activation window

        while traci.simulation.getTime() < end_time:
            t = traci.simulation.getTime()
            traci.simulationStep()
            rsu_manager.step()
            manager.step(t)

            # Sample each degraded edge every step
            for e in degraded_edges:
                try:
                    vids = traci.edge.getLastStepVehicleIDs(e)
                except Exception:
                    vids = []
                for vid in vids:
                    try:
                        _diag_total[e] += 1
                        if traci.vehicle.getVehicleClass(vid) != "passenger":
                            continue  # Gate A: only sample passenger vClass (ego's class)
                        f_rate = traci.vehicle.getFuelConsumption(vid)   # mg/s
                        spd    = traci.vehicle.getSpeed(vid)              # m/s
                        if f_rate > 0:
                            _diag_nonzero[e] += 1
                        if f_rate > 0 and spd > 0.1:
                            if t <= degrade_start:
                                fuel_before[e].append(f_rate)
                                spd_before[e].append(spd)
                            else:
                                fuel_after[e].append(f_rate)
                                spd_after[e].append(spd)
                    except Exception:
                        pass

            # After 1000 steps print a one-time diagnostic
            if not _diag_printed and t >= 15000:
                _diag_printed = True
                print("\n[Gate A diagnostic @ t=15000]")
                for e in degraded_edges:
                    try:
                        vids = traci.edge.getLastStepVehicleIDs(e)
                    except Exception:
                        vids = []
                    print(f"  {e}: {len(vids)} vehicles on edge right now, "
                          f"cumulative veh-steps={_diag_total[e]}, "
                          f"nonzero-fuel steps={_diag_nonzero[e]}")
                    for vid in list(vids)[:3]:
                        try:
                            ec  = traci.vehicle.getEmissionClass(vid)
                            vcc = traci.vehicle.getVehicleClass(vid)
                            fc  = traci.vehicle.getFuelConsumption(vid)
                            spd = traci.vehicle.getSpeed(vid)
                            print(f"    vid={vid} vClass={vcc} emClass={ec} fuel={fc:.3f}mg/s spd={spd:.2f}m/s")
                        except Exception as ex:
                            print(f"    vid={vid} error={ex}")

        traci.close()

        print("\nGate A Results (passenger vClass only — matching ego vehicle class):")
        for e in degraded_edges:
            L = calc.edge_lengths.get(e, 100.0)
            # fuel/m = mean(fuel_rate / speed)  [mg/s / (m/s) = mg/m]
            def fuel_per_m(rates, spds):
                samples = [r/s for r, s in zip(rates, spds) if s > 0]
                return sum(samples)/len(samples) if samples else 0.0, len(samples)

            fb, nb = fuel_per_m(fuel_before[e], spd_before[e])
            fa, na = fuel_per_m(fuel_after[e],  spd_after[e])
            # traversal time = L / mean_speed
            tb = L / (sum(spd_before[e])/len(spd_before[e])) if spd_before[e] else 0.0
            ta = L / (sum(spd_after[e])/len(spd_after[e]))   if spd_after[e]  else 0.0

            f_ratio = (fa / fb) if fb > 0 else 0.0
            t_ratio = (ta / tb) if tb > 0 else 0.0


            fuel_pass = f_ratio >= 1.30
            time_pass = t_ratio <= 1.05  # time rise <= ~5% is the hard requirement

            print(f"  Edge {e}:  (samples before={nb}, after={na})")
            print(f"    Fuel/m : Before={fb:.3f} mg/m, After={fa:.3f} mg/m -> Ratio = {f_ratio:.2f}x")
            print(f"    Time   : Before={tb:.1f} s,    After={ta:.1f} s    -> Ratio = {t_ratio:.3f}x")
            print(f"    -> Fuel PASS" if fuel_pass else f"    -> Fuel FAIL (need >=1.30x, got {f_ratio:.2f}x)")
            print(f"    -> Time PASS" if time_pass else
                  f"    -> Time FAIL (need <=1.05x, got {t_ratio:.2f}x) "
                  f"[mechanism is slowing vehicles -> not grade mode]")

            if fuel_pass and time_pass:
                print(f"    => Gate A PASS for {e}")
            else:
                print(f"    => Gate A FAIL for {e}")

        sys.exit(0)

    # Fix 3: generate a plain OD list first so the nonbottleneck selector can use
    # real campaign ODs for coverage filtering, then (if targeted-od) re-generate.
    if _nonbottleneck_deferred:
        _plain_od = generate_od_pairs(NET_FILE, n=args.n, seed=args.od_seed)
        degraded_edges = select_nonbottleneck_grade_edges(
            NET_FILE, CONFIG_FILE, k=args.n_degraded,
            probe_seed=args.traffic_seed,
            sumo_bin=SUMO_BIN, scale=args.scale, teleport=args.teleport,
            od_list=_plain_od,
            probe_begin=max(0, depart_start - 300),
        )
        print(f"[AUTO_NONBOTTLENECK] selected edges: {degraded_edges}")

    if args.targeted_od and degraded_edges:
        od_list = select_targeted_od_pairs(NET_FILE, degraded_edges, n=args.n,
                                           seed=args.od_seed,
                                           max_detour_factor=args.max_detour_factor,
                                           max_trip_time=args.max_od_trip_time)
    else:
        od_list = generate_od_pairs(NET_FILE, n=args.n, seed=args.od_seed)

    if not od_list:
        sys.exit("No OD pairs generated; aborting.")

    # Same seed + same OD list for both arms, so the only systematic difference
    # is who routes the ego. Parallel and sequential give identical results.
    jobs = [("sumo", "sumo"), ("ours", "ours")]

    # D4: only import/construct when degradation is requested; otherwise pass None so
    # all existing modes (no road-condition flags) behave exactly as before.
    if args.road_condition != "none" and degraded_edges:
        from Simulation.road_conditions import RoadConditionManager
        manager = RoadConditionManager(
            degraded_edges, args.road_condition, degrade_start,
            v_low=args.degrade_vlow, v_high=args.degrade_vhigh,
            period_s=args.degrade_period, event_duration=args.event_duration,
            grade_emission_class=args.grade_emission_class
        )
    else:
        manager = None
    if args.paired:
        print("\n[Paired Mode] Running fixed-departure continuous fuel campaign.")
        seeds = [int(x.strip()) for x in args.seeds.split(",")]
        run_meta = {
            "n": args.n, "od_seed": args.od_seed, "seeds": seeds,
            "scale": args.scale, "teleport": args.teleport,
            "alpha": ALPHA, "beta": BETA, "gamma": GAMMA,
            "depart_start": args.depart_start, "depart_spacing": args.depart_spacing,
            "reroute_interval": args.reroute_interval, "use_hysteresis": args.use_hysteresis,
            "baseline": args.baseline,
            # D5: Gate C needs these to compute avoidance rates
            "degraded_edges": degraded_edges,
            "road_condition": args.road_condition,
        }
        
        if args.arms:
            # Change 3: explicit --arms flag overrides --baseline
            arms_to_run = [a.strip() for a in args.arms.split(",") if a.strip()]
            print(f"[arms] Using explicit --arms: {arms_to_run}")
        else:
            # Legacy: build from --baseline (two-arm, byte-compatible)
            arms_to_run = ["ours"]
            if args.baseline in ["sumo", "both"]: arms_to_run.append("sumo")
            if args.baseline in ["ablation", "both"]: arms_to_run.append("ablation")
        
        scales = [float(x.strip()) for x in args.scale_sweep.split(",")] if args.scale_sweep else [args.scale]
        departs = [float(x.strip()) for x in args.depart_sweep.split(",")] if args.depart_sweep else [args.depart_start]
        
        sweep_results = []
        import csv
        
        for scale in scales:
            for depart in departs:
                print(f"\n=======================================================")
                print(f" SWEEP POINT: scale={scale}, depart={depart}")
                print(f"=======================================================")
                all_res = {arm: [] for arm in arms_to_run}
                cfs_all = []
                route_cfs_all = []

                run_meta["scale"] = scale
                run_meta["depart_start"] = depart
                diag_stamp = time.strftime("%Y%m%d_%H%M%S")

                # --seed-parallelism P: run P seeds concurrently; each seed still runs
                # its two arms in parallel within the same pool (2*P total workers).
                _seed_p = max(1, args.seed_parallelism)
                _max_workers = _seed_p * len(arms_to_run)
                print(f"[seed-parallelism] P={_seed_p}, arms={len(arms_to_run)}, "
                      f"total_workers={_max_workers}")
                ctx = multiprocessing.get_context("spawn")
                ex = concurrent.futures.ProcessPoolExecutor(
                    max_workers=_max_workers, mp_context=ctx)

                futs = {}
                for seed in seeds:
                    print(f"\n--- Queuing SEED {seed} ---")
                    seed_diag = f"{diag_stamp}_{seed}"   # unique diag file per seed
                    for arm in arms_to_run:
                        a, b, g, arm_cost_mode = _arm_params(arm, args)
                        is_ours = arm not in ("ablation", "sumo")
                        fut = ex.submit(
                            _run_paired_capture, arm, a, b, g, od_list, seed,
                            f"{arm}_{seed}_{scale}_{depart}",
                            depart, args.depart_spacing, args.use_hysteresis,
                            scale, args.teleport,
                            road_condition_manager=manager,
                            debug_cfs=(args.debug_cfs and is_ours),
                            diag_stamp=seed_diag,
                            theta_fuel=args.theta_fuel if is_ours else 1.0,
                            theta_time=args.theta_time,
                            cost_mode=arm_cost_mode,
                            fuel_aggregator=args.fuel_aggregator if is_ours else "median",
                            fuel_sample_max_age_s=600.0,
                            junction_weight=args.junction_weight if is_ours else 0.0,
                            fuel_hysteresis=args.fuel_hysteresis if is_ours else 0.0,
                        )
                        futs[fut] = (seed, arm)

                for fut in concurrent.futures.as_completed(futs, timeout=WORKER_TIMEOUT_S):
                    seed_done, arm_done = futs[fut]
                    out = fut.result()
                    print(f"\n# scenario '{out['tag']}' (seed={seed_done}) finished in {out['wall_s']:.1f}s")
                    if out["error"]: print(f"[ERROR] {out['error']}")
                    if out.get("log"): print(out["log"], end="" if out["log"].endswith("\n") else "\n")
                    all_res[arm_done].append(out["results"])
                    if arm_done == "ours" and args.debug_cfs:
                        cfs_all.extend(out.get("cfs_records", []))
                        route_cfs_all.extend(out.get("route_cfs_records", []))

                ex.shutdown(wait=False, cancel_futures=True)
                    
                sum_json = None
                # Change 3: multi-arm analysis — compare every non-reference arm against ablation;
                # legacy two-arm mode (ours vs sumo/ablation) is byte-compatible.
                if "sumo" in all_res and "ours" in all_res:
                    analyze_paired_results(all_res["ours"], all_res["sumo"], "sumo", run_meta)
                if "ablation" in all_res:
                    ref = all_res["ablation"]
                    for arm_name in ("ours", "ours-fuel", "ours-augtime"):
                        if arm_name in all_res:
                            sum_json = analyze_paired_results(all_res[arm_name], ref, "ablation", run_meta,
                                                              ours_label=arm_name)
                    
                if args.debug_cfs and cfs_all:
                    stamp = time.strftime("%Y%m%d_%H%M%S")
                    import csv
                    with open(f"cfs_debug_{stamp}.csv", "w", newline="") as f:
                        w = csv.DictWriter(f, fieldnames=list(cfs_all[0].keys()))
                        w.writeheader()
                        w.writerows(cfs_all)
                    if route_cfs_all:
                        with open(f"route_cfs_{stamp}.csv", "w", newline="") as f:
                            w = csv.DictWriter(f, fieldnames=list(route_cfs_all[0].keys()))
                            w.writeheader()
                            w.writerows(route_cfs_all)
                    
                    # Compute summary
                    total_calls = len(cfs_all)
                    bl_locked = sum(1 for r in cfs_all if r["baseline_locked"])
                    max_route_mult = max((r["multiplier"] for r in route_cfs_all), default=1.0)
                    
                    print("\n" + "="*50)
                    print(" PHASE 1 DEBUG-CFS SUMMARY")
                    print("="*50)
                    print(f"Total compute_weight calls: {total_calls}")
                    print(f"Evaluated edges with baseline_locked: {bl_locked}/{total_calls} ({bl_locked/total_calls*100:.2f}%)")
                    
                    bins = {"[1.00,1.01)": 0, "[1.01,1.05)": 0, "[1.05,1.10)": 0, "[1.10,1.25)": 0, "[1.25,+)": 0}
                    for r in cfs_all:
                        m = r["multiplier"]
                        if m < 1.01: bins["[1.00,1.01)"] += 1
                        elif m < 1.05: bins["[1.01,1.05)"] += 1
                        elif m < 1.10: bins["[1.05,1.10)"] += 1
                        elif m < 1.25: bins["[1.10,1.25)"] += 1
                        else: bins["[1.25,+)"] += 1
                    
                    print("Multiplier distribution:")
                    for k, v in bins.items(): print(f"  {k}: {v}")
                    
                    C_vals = [r["C"] for r in cfs_all]
                    F_vals = [r["F"] for r in cfs_all]
                    S_vals = [r["S"] for r in cfs_all]
                    
                    def p_stat(vals):
                        if not vals: return 0,0,0,0
                        s = sorted(vals)
                        return sum(vals)/len(vals), s[len(s)//2], s[int(len(s)*0.95)], s[-1]
                        
                    cm, cp50, cp95, cmax = p_stat(C_vals)
                    fm, fp50, fp95, fmax = p_stat(F_vals)
                    sm, sp50, sp95, smax = p_stat(S_vals)
                    print(f"\nStats (mean / p50 / p95 / max):")
                    print(f"  C: {cm:.4f} / {cp50:.4f} / {cp95:.4f} / {cmax:.4f}")
                    print(f"  F: {fm:.4f} / {fp50:.4f} / {fp95:.4f} / {fmax:.4f}")
                    print(f"  S: {sm:.4f} / {sp50:.4f} / {sp95:.4f} / {smax:.4f}")
                    
                    print(f"\nMax multiplier on any route (max_route_multiplier): {max_route_mult:.4f}")
                    
                    if max_route_mult <= 1.025:
                        print(">>> VERDICT: H1 confirmed. Penalty is effectively dead. The +0.00% tie is an artifact.")
                    elif max_route_mult > 1.10:
                        print(">>> VERDICT: H2 supported. Penalty is alive but optimal paths coincide.")
                    else:
                        print(">>> VERDICT: Middle case. Penalty is present but weak (1.02-1.10).")
                        
                if sum_json and (args.scale_sweep or args.depart_sweep):
                    with open(sum_json) as f:
                        j = json.load(f)
                    
                    mean_net_speed = 0.0
                    mean_tl = 0.0
                    n_trips = 0
                    for arm_trips in all_res["ours"]:
                        for trip in arm_trips:
                            if trip.get("arrived"):
                                mean_net_speed += (trip.get("routeLength", 0) / trip.get("duration", 1))
                                mean_tl += trip.get("timeLoss", 0)
                                n_trips += 1
                    if n_trips > 0:
                        mean_net_speed /= n_trips
                        mean_tl /= n_trips
                        
                    f_stats = j.get("fuel_stats", {})
                    
                    sweep_results.append({
                        "scale": scale,
                        "depart_start": depart,
                        "mean_network_speed": mean_net_speed,
                        "mean_timeLoss": mean_tl,
                        "K_seeds": j["run_meta"]["K_seeds"],
                        "mean_fuel_saving_pct": j.get("fuel_sav_mean", 0),
                        "ci_low": f_stats.get("CI_95_pct", [0,0])[0],
                        "ci_high": f_stats.get("CI_95_pct", [0,0])[1],
                        "ttest_p": f_stats.get("t_test_p", 1.0),
                        "wilcoxon_p": f_stats.get("wilcoxon_p", 1.0),
                        "mean_time_saving_pct": j.get("dur_sav_mean", 0),
                        "mean_reroutes": sum(t.get("reroutes", 0) for arm_t in all_res["ours"] for t in arm_t) / (len(all_res["ours"])*len(od_list))
                    })
        
        if sweep_results:
            stamp = time.strftime("%Y%m%d_%H%M%S")
            with open(f"congestion_sweep_{stamp}.csv", "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=list(sweep_results[0].keys()))
                w.writeheader()
                w.writerows(sweep_results)
                
            print("\n=======================================================")
            print(" PHASE 2 SWEEP RESULTS ")
            print("=======================================================")
            print(f"{'Scale':<8} | {'Depart':<8} | {'Speed':<8} | {'TimeLoss':<8} | {'FuelSav%':<8} | {'TimeSav%':<8} | {'Reroutes':<8}")
            for r in sweep_results:
                print(f"{r['scale']:<8.2f} | {r['depart_start']:<8.0f} | {r['mean_network_speed']:<8.2f} | {r['mean_timeLoss']:<8.2f} | {r['mean_fuel_saving_pct']:<8.2f} | {r['mean_time_saving_pct']:<8.2f} | {r['mean_reroutes']:<8.2f}")
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
