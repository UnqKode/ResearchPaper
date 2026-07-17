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
