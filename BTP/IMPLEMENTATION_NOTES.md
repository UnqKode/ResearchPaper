# Implementation Notes — Round 3

## Fix A: libsumo compat shim (`traci_compat.py`)

`BTP/traci_compat.py` is the single import point for the TraCI API. It prefers
`libsumo` (in-process C++ call, no socket overhead) and falls back to network
`traci` on `ImportError`. Set `FORCE_TRACI=1` to force network TraCI (needed for
`sumo-gui` or multi-client debugging).

**libsumo constraints:** one simulation per process; no GUI. Both are satisfied
because the multiprocessing pool uses Windows' default `spawn` start method —
each worker process gets a fresh libsumo instance with no cross-process state.

**Current status:** `libsumo` is not installed in the `ml` conda environment as
of Round 3, so all runs fall back to network TraCI. Replacing it delivers a
predicted 10–50× per-step speedup; install with `pip install libsumo==1.27.0`
(matching SUMO 1.27.0). The fallback is transparent — no code changes needed.

**API guard:** `traci.start(cmd, port=, label=)` — the `port` and `label`
arguments are ignored by libsumo (one sim per process). All `traci.start()` call
sites are guarded: `if USING_LIBSUMO: traci.start(cmd) else: traci.start(cmd, port=port)`.
The pre-start stale-connection `traci.close()` is also guarded to
`not USING_LIBSUMO` (libsumo raises on close with no active sim).

---

## Fix B: Vehicle sampling (`SAMPLE_MOD = 5`)

**Problem:** The original code subscribed every vehicle that entered any tracked
edge via `traci.vehicle.subscribe()`. With all 4,404 Monaco edges tracked and
thousands of concurrent vehicles, each `simulationStep()` response carried data
for all subscribed vehicles — growing without bound as the campaign progressed.

**Solution:** Only ~20% of background vehicles are subscribed at any time (1 in
every `SAMPLE_MOD = 5`, selected by `zlib.crc32(vid) % SAMPLE_MOD == 0`).

**Honesty constraint:** Sampling is deterministic by vehicle ID hash only. It is
completely blind to edge identity, arm identity, RoadConditionManager state, and
whether the vehicle is on a degraded or non-degraded edge. This ensures the
per-edge fuel signal has no systematic bias from the sampling scheme.

**Ego vehicles** (`vid.startswith("ego_")`) are always subscribed regardless of
hash, so ego metrics (fuel, reroutes, driven edges) are complete.

**Denominator:** Stop-probability and per-traversal fuel are computed only from
sampled vehicles' completed traversals. The `vehicle_count` in RSU data points
counts sampled-departed vehicles only. This means the window fills ~5× slower
at `SAMPLE_MOD=5`, but the cold-fallback behaviour in `EdgeCostCalculator`
covers sparse edges as designed. A startup log line reports:
`[RSU] vehicle sampling 1/5: expect ~5x slower fuel-window fill`.

Set `--vehicle-sample-mod 1` to restore the old behaviour (subscribe everything).

---

## Fix C: Cached socket calls

`rsu_manager.getDeltaT()` is called once in `subscribe_edges()` and cached as
`self._dt`. `rsu_manager.step()` accepts `sim_time` from the caller (which
already reads it once per step in the outer loop) instead of calling
`traci.simulation.getTime()` internally. This eliminates two per-step socket
round-trips that cost ~10 ms each on Windows localhost TCP.

---

## Fix D: Per-seed saveState warmup

**Tradeoff:** Previously, all three arms independently re-simulated the full
14,400 → 21,600 warm-up (~7,200 sim-seconds) identically (since no egos are
active yet). This was a 3× waste — and the bottleneck that burned 16 wall-clock
hours in Round 2.

**Solution:** One headless SUMO instance per seed runs the warmup (14400 →
`depart_start - 600` = 21,000), saves the traffic state with
`traci.simulation.saveState(...)`, then closes. Each arm's SUMO starts with
`--load-state` pointing at that file and `--begin` matching the saved time.
SUMO resumes at that exact traffic state; the arm only simulates
21,000 → campaign end, not 14,400 → campaign end.

**What saveState does NOT save:** Python-side RSU state. Each arm's
`RSUManager`, `EdgeCostCalculator`, and `RoadConditionManager` start cold at
`depart_start - 600`. The 600-second buffer gives the RSU time to collect
baseline fuel samples on the degraded edges before egos depart. If baselines
don't lock in 600 s (check for `[BASELINE_LOCK]` log lines), increase the
buffer to 1,200 s via `depart_start - 1200`.

**Grade-mode timing:** `RoadConditionManager` triggers off absolute sim time
(`rcm.activate_time = 21,300`). Since SUMO resumes at t=21,000, activation at
21,300 fires 300 sim-seconds into each arm — unchanged from the non-saveState
behaviour.

**State file names:** `warmstate_{seed}_{scale}_{depart}.xml.gz` in the BTP/
working directory. Deleted after each seed unless `--keep-warmstate` is passed.

**Determinism check:** SUMO state loading from the same binary and config is
deterministic. A `[STATE_DETERMINISM]` check per seed (comparing background
vehicle counts at a checkpoint across arms) is planned for a future round.

---

## Fix E: Corridor validity gate

**Problem fixed:** `f8e3ede` (the sumolib-only structural selector) dropped the
non-bottleneck free-flow check — the scientific core of the experiment. Edges
were selected blind on speed limit, length, and OD coverage only, with no
verification that they were actually free-flowing under realistic demand.

**Solution:** The structural selector now returns `k=6` candidates. During each
seed's phase-0 warmup, their occupancy and speed are measured over the last
1,800 sim-seconds (the `CORRIDOR_MEASURE_WINDOW_S`).

Gate thresholds:
- `speed_ratio = avg_speed / speed_limit ≥ 0.85` (free-flow, not congested)
- `avg_occ ≤ 0.40` (normalised, not bottleneck)

Passing candidates are ranked by speed_ratio (desc), then length (desc). The
top 2 become `degraded_edges`. If fewer than 2 pass: `[CORRIDOR_GATE] FAILED`
is printed with measured values and the seed aborts unless `--force-corridor` is
passed. Self-healing: with 6 structural candidates, the gate turns from
abort-only into select-best-2.

---

## Fix F: Arm-params integrity assertion

`a1f26ae` maps all non-sumo arms to `ego_routing="ours"`, which is correct but
creates a seam where `cost_mode` and `alpha/beta/gamma` must still differ
correctly per arm. The assertion in `run_paired_scenario` catches any wiring
bug at startup:
- `ours-fuel` → `cost_mode=fuel`
- `ours-augtime` → `cost_mode=augtime`, `(α,β,γ)=(1.0,8.0,10.0)`
- `ablation` → `cost_mode=augtime`, `(α,β,γ)=(0.0,0.0,0.0)`

The `[ARM_CONFIG]` log line is printed before the assertion for every arm,
giving a diagnostic trail even if the assertion doesn't fire.

---

## Fix G: Step-rate telemetry

Every 500 steps of `run_fixed_departure_campaign` (post fast-forward), a
`[PERF]` line is written to stderr:
```
[PERF] seed=1 arm=ours-fuel sim_t=21850 steps/s=47.3 active_veh=2
```
This bypasses `redirect_stdout` and is visible immediately in the err log.
The acceptance criterion for the Round-3 fixes is ≥ 30 steps/s sustained
in the post-19500 heavy phase. During fast-forward (`sim_t < 19500`), the
step counter is not incremented so the rate reflects only the heavy phase.

---

# Implementation Notes — Round 4

## Fix 1: Warmup buffer 600 → 1200 s (`--warmup-buffer`)

**Problem fixed:** With only 300–600 s of RSU warmup before grade activation, low-traffic
degraded edges (e.g. 0.43 veh/min at scale=2.0) had fewer than 3 baseline samples. The
`_fuel_baseline` was `None` at `freeze_baseline()` time, so `F=0` for all edges and
ours-fuel/ours-augtime/ablation produced identical routing → 0% delta.

**Solution:** `--warmup-buffer 1200` (default). `warmup_end = depart_start - 1200 = 20400`.
This gives the RSU 1200 s to build baselines before grade activates at 21000.

**Expected samples:** At 0.43 veh/min and 1/2 sampling, 1200 s → ~4.3 expected samples on
the least-trafficked degraded edge. Combined with the EMA seed buffer (5 observations
required before locking), this is still tight — but grade-lead=600 gives additional time.

---

## Fix 2: Vehicle sampling 1/5 → 1/2 (`--vehicle-sample-mod 2`)

**Why:** At `sample_mod=5`, only 20% of background vehicles contributed traversal records.
On a low-traffic corridor edge, this extended the baseline fill time to ~25 min at the
Monaco scale. Halving to 1/2 doubles the effective sample rate with minimal step-rate
impact (subscribed vehicle count doubles, but only 1/4 of the original 4404 edges are
subscribed at once, so the per-step payload is bounded).

**Honesty constraint unchanged:** sampling is still `zlib.crc32(vid.encode()) % sample_mod == 0`,
blind to edge identity, arm identity, and RoadConditionManager state.

---

## Fix 3: Grade lead 300 → 600 s (`--grade-lead`)

**Why:** Grade activates at `depart_start - grade_lead`. With `grade_lead=300`, the
300-second window (t=21300→21600) was used for BOTH baseline-seeding AND grade-active
observation — a race condition. With `grade_lead=600`, grade activates at t=21000 and
RSU has 600 s of free-flow fuel data locked before egos depart.

**Invariant preserved:** grade_lead < warmup_buffer (600 < 1200), so RSU starts observing
before grade fires.

---

## Fix 4: Baseline-lock gate (`[BASELINE_GATE]`) + `[FUEL_WINDOW]`

**`[BASELINE_GATE]`:** `RoadConditionManager._verify_baseline_locks()` now prints
`[BASELINE_GATE] PASS/FAILED` per degraded edge at grade activation time. On failure it
raises `RuntimeError`, aborting the arm (caught by `_run_paired_capture`, marked as error).
Pass `--force-baseline` to demote this to a warning and continue anyway.

**`[FUEL_WINDOW]`:** Printed at the first ego departure. Shows `n_traversals`, `baseline`,
and `frozen` status for each degraded edge. Lets you verify that baselines were locked
before egos start routing.

---

## Fix 5: Reroute-acceptance telemetry (`[REROUTE_SUMMARY]`)

At end of each arm campaign:
```
[REROUTE_SUMMARY] arm=ours-fuel seed=1 reroute_evals=47 accepted=3 best_rejected_margin=8.4%
```
- `reroute_evals`: how many times the reroute decision fired (Dijkstra ran)
- `accepted`: how many were accepted (route switch happened)
- `best_rejected_margin`: highest saving% that was still below hysteresis threshold

`[FUEL_HYST]` lines (per reroute decision) are already emitted in `_reroute_fuel_mode`
and now include `margin=X%` on KEEP actions.

---

## Fix 6: End-of-arm PERF summary (`[PERF_SUMMARY]`)

At end of each arm campaign (to stderr):
```
[PERF_SUMMARY] arm=ours-fuel seed=1 total_steps=8240 mean_steps_s=18.3 peak_active_veh=5 wall_s=450.1s
[PERF_SUMMARY] projected k=10: ~1.25h total (10 seeds serial, 3 arms parallel/seed)
```
The projection assumes all seeds run serially and all 3 arms run in parallel per seed
(the default `--seed-parallelism 1` configuration). Formula:
`projected_total = 10 × arm_wall_s / 3600` hours.

---

## Fix J: Freeze-baseline cold pre-seed (Round 4)

`freeze_baseline()` previously pre-seeded `_fuel_baseline[edge]` from the network median
only for degraded edges (called from `RoadConditionManager._activate()` over
`self.degraded_edges`). This is superseded by Fix L.

---

# Runbook

## Never edit source while a campaign is running

The campaign uses a `multiprocessing.ProcessPoolExecutor` with `spawn` context. Worker
processes **import all Python modules once at spawn time** and reuse that in-memory module
for all subsequent seeds. Editing a source file mid-run is invisible to already-spawned
workers and guarantees inconsistent behaviour across seeds (some seeds run old code,
some run new).

**Rule:** Any fix, however small, requires: **kill campaign → edit → commit → relaunch**.
A mid-run edit is a silent correctness hazard, not just a cosmetic risk.

---

# Implementation Notes — Round 5

## Fix L: Edge-blind `preseed_cold_baselines()` (honesty fix for Fix J)

**Problem:** `freeze_baseline()` was called from `RoadConditionManager._activate()` over
`self.degraded_edges` only. The pre-seed for cold edges (added in Fix J / `903dbed`)
therefore fired only on degraded edges — a corridor-conditioned treatment that created
an asymmetry between degraded and non-degraded cold edges in routing-input state.
Specifically, a cold degraded edge received a synthetic baseline while an equally cold
non-degraded edge did not, biasing fuel-mode routing against the degraded corridor before
any real EU0 signal accumulated.

**Fix:** `EdgeCostCalculator.preseed_cold_baselines(sim_time)` pre-seeds ALL edges in
`edge_lengths` that have no locked baseline, using `cold_nominal_rate()` (network-median
of observed locked baselines, or the hardcoded 50 mg/s fallback before any lock).  It
does NOT read `degraded_edges` and does NOT live inside `RoadConditionManager`.

**Call site:** `simulate.py::run_fixed_departure_campaign`, fires once when
`sim_time >= rcm.activate_time` (the grade-activation timestamp), BEFORE
`road_condition_manager.step()` triggers `freeze_baseline()` on degraded edges.

**`freeze_baseline()` change:** The cold pre-seed block removed. `freeze_baseline()` now
just adds `edge_id` to `_frozen_baseline_edges` and logs the existing baseline (which
was either observed or set by `preseed_cold_baselines`).

**Log line:** `[PRESEED] cold baselines pre-seeded for N/M edges at t=21300 (rate=...mg/s)`

**Invariant verified by unit tests:** `degraded` and `clean` cold edges receive the
identical baseline value at preseed time. `freeze_baseline` on the degraded edge does
not change the value — it only locks further EMA updates.

---

## Fix M: Corridor gate — traversal-count criterion replaces occupancy floor

**Problem:** With `--device.rerouting.probability=1`, Monaco MoST traffic disperses
so completely that the probe measured max avg_occ = 0.0001 across all 2279 edges in the
5–30 m/s, 50m+ category. `CORRIDOR_OCC_MIN=0.005` therefore ALWAYS rejected every
candidate, and the force-fallback silently ran on every seed. A gate that can never
pass is dead code.

**Fix:** Replace `CORRIDOR_OCC_MIN` (removed) with `CORRIDOR_MIN_TRAVERSALS = 5`.
The warmup phase now subscribes to `LAST_STEP_VEHICLE_ID_LIST` for each candidate edge
and counts sampled vehicle exits (filtered by `zlib.crc32(vid.encode()) % vehicle_sample_mod == 0`)
during the last `CORRIDOR_MEASURE_WINDOW_S` seconds. A candidate PASSES when:
- `warmup_traversals >= CORRIDOR_MIN_TRAVERSALS` (evidence: ≥5 sampled ≈ ≥10 real at mod=2)
- `speed_ratio >= CORRIDOR_SPEED_RATIO_MIN` (free-flow, not congested)
- `avg_occ <= CORRIDOR_OCC_MAX` (upper bound against bottleneck, retained)

**Return value change:** `_corridor_gate()` now returns `(degraded_edges, gate_status)`
where `gate_status ∈ {"passed", "fallback"}`. `_run_warmup_phase()` returns
`(degraded_edges, warmup_end_time, gate_status)`. Per-seed gate status is recorded in
the analysis JSON as `per_seed_corridor_gate: {"1": "passed", "2": "fallback", ...}`.

**Force-fallback log:** When fewer than 2 candidates pass and `--force-corridor` is set,
the fallback prints `[CORRIDOR_GATE] WARNING: fallback used — gate did not pass on any
candidate` so it is visibly distinguishable from a passed run.

**Deviations from spec:** None.
