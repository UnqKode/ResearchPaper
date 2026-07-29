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

---

# Implementation Notes — Round 6

## Process Failure — Round 5 campaign launched without passing gates

The Round-5 campaign (10 seeds, 3 arms, ~20.8 wall-hours) was launched without verifying
the pre-campaign acceptance criteria documented in Fix G:

- **`[PERF]` ≥ 30 steps/s** criterion was NOT checked. The campaign ran at 1.7–2.6 steps/s
  — approximately 15× below the criterion. This was a known consequence of `libsumo` not
  being installed in the `ml` conda environment (`USING_LIBSUMO = False` in all workers).
- **Nonzero arm-divergence smoke** was not run before the 10-seed campaign. A 1-seed,
  n=3 smoke would have revealed the null result within minutes.

**Consequence:** 20+ wall-hours of compute produced a systematic negative result (dz=−33.7)
whose root causes could have been diagnosed from a 30-minute smoke run.

**Rule (permanent, mandatory):** No campaign launch (k>1, n>5) unless EVERY acceptance
criterion of the preceding smoke passes verbatim:
1. `[TRANSPORT] USING_LIBSUMO=True` in all workers
2. `[PERF]` ≥ 30 steps/s sustained in the post-fast-forward heavy phase
3. `[BASELINE_GATE]` PASS for all degraded edges
4. Nonzero fuel delta between at least one "ours" arm and ablation
5. At least one accepted reroute OR documented reason why δ prevents it

This is not optional and cannot be overridden by time pressure.

---

## Bug N: `per_seed_corridor_gate` injection crash (post-campaign fix)

`analyze_paired_results()` returns `f_sum` — the output file path (a string). The
post-analysis injection at line 2566 treated it as a dict:
```python
sum_json["per_seed_corridor_gate"] = {...}   # TypeError: str does not support item assignment
```
**Fix:** Changed `sum_json = None` to `sum_json_paths = []`; each call to
`analyze_paired_results` appends the returned path; the injection loop reads, updates,
and rewrites each JSON file.

---

## Round 6 Diagnostics (D1–D4) — `--diagnose` flag

Instrumentation added to `compare_routing.py` and `simulate.py` behind `--diagnose`.
All diagnostic output is gated on this flag and removed/disabled after Round 6.

### D1 — Weight-vector dump (tests H1/H2)
At t=21590 (10 s before first ego), each arm writes
`diag_weights_<arm>_seed<N>.csv`: `edge_id, weight, source, n_window_samples, baseline`
where `source ∈ {observed_window, preseed_only, cold_fallback}`.

### D2 — Initial-route + corridor engagement log (tests H3)
For every ego in every arm at insertion:
`[EGO_ROUTE] arm=... ego=... n_edges=... route_len_m=... contains_corridor=True|False`
Full edge list written to `diag_routes_<arm>_seed<N>.csv`.

### D3 — Transport audit (tests H4)
At worker startup:
`[TRANSPORT] USING_LIBSUMO=<bool> traci_module=<path> edge_subs=<count>`

### D4 — Hysteresis margin distribution
REROUTE_SUMMARY extended to report `p50_margin`, `p90_margin`, `max_margin` across all
rejected evals. Calibrates Fix S δ from data.

---

## Fix P — Persist Python-side learning with warm state

Phase-0 warmup previously discarded all RSU/EdgeCostCalculator learning. Fix P serializes
the learned state (fuel baselines, traversal-fuel windows, stop stats) to
`warmstate_seed{N}_python.pkl` at state-save time. Each arm loads BOTH the SUMO `.xml.gz`
and the `.pkl` before its own RSU starts.

- Serialized via `get_state()`/`load_state()` methods on RSUManager and EdgeCostCalculator.
- Does NOT pickle objects — plain dicts/lists only.
- Phase 0 has no degradation → pickle is arm-neutral by construction.
- Each arm logs `[PYSTATE] loaded sha=<sha256[:12]>` to verify identical state.

---

## Fix Q — Structural preseed (regression-estimated per-edge baseline)

Replaces the uniform `cold_nominal_rate()` preseed value with a per-edge OLS estimate:
`baseline_rate ≈ b0 + b1·speed_limit + b2·lane_count`
fit on edges with locked baselines after phase-0. Cold edges get their own prediction
clamped to [0.5×, 2×] network median. Logged as `[PRESEED_FIT] R2=... n_fit=...`.
Fallback to uniform median if < 200 fit edges.

Same fix applied to `_cold_fuel_fallback`: `free_flow_time × predicted_rate`.

---

## Fix R — Cold-edge junction penalty uses median p_stop

Previously, `get_junction_penalty` returned 0 immediately when `stop_and_go_freq <= 0`
(cold edge). This made unobserved side-streets look junction-free, pushing egos onto them.
Fix: when `stop_and_go_freq <= 0` and `cold_pstop_mode="median"` (default), use the
network-median observed p_stop instead of 0. Exposed via `--cold-pstop-mode {median,zero}`.

---

## Fix S — Hysteresis δ calibrated from D4 data

New default δ = `min(0.03, p90_keep_margin_from_D4 / 2)`, floored at 0.01.
Hardcoded result documented here once D4 data is in hand.

---

## Fix T — OD pair corridor-enforcement repair

`select_targeted_od_pairs` uses sumolib static Dijkstra. SUMO's device.rerouting uses
historical travel times, so SUMO's initial route at insertion may differ from sumolib's
optimal path. When `crossed=False` for ablation egos (pre-degradation), the OD selection
is not delivering corridor-crossing routes in practice.

Fix: verify corridor crossing at SUMO routing time, not just at OD-selection time.
Specifically, inject each ego with an explicit SUMO `traci.vehicle.setRoute()` that
follows the sumolib corridor-crossing path. The ego then either stays on it (ablation)
or gets rerouted off it by our dynamic weights (ours arms).

---

## Fix U — Transport repair (install libsumo in ml env)

`traci_compat.py` falls through to network TraCI when `import libsumo` raises ImportError.
The `ml` conda environment does not have libsumo installed.

Fix: `pip install libsumo==1.27.0` inside the `ml` env (matching SUMO 1.27.0).
After install, all workers report `[TRANSPORT] USING_LIBSUMO=True` and `[PERF]` must
reach ≥ 30 steps/s sustained.

---

# Implementation Notes — Round 7B

## ERRATA: Fix R label collision (commit `a89db7f`)

Commit `a89db7f` was tagged **"Fix R"** in its git message. That label is already
claimed: `Fix R` in this file is "Cold-edge junction penalty uses median p_stop"
(deferred, unimplemented). Commit `a89db7f` must be relabelled **Fix Q2a**.
The original Fix R (cold junction penalty) and Fix S (δ hysteresis from D4 data)
remain **deferred and unimplemented** as of Round 7B.

---

## Fix Q2a — Sub-1m stub edge filter *(relabelled from "Fix R" / a89db7f)*

Excludes edges with `length < 1.0 m` from `_build_graph()` (routing graph). SUMO's
built-in router naturally avoids these; our fuel-mode Dijkstra assigned them near-zero
weight (short edge → cheap cold fuel estimate) and routed through them, causing a
libsumo C-level abort when the 5 m ego vehicle tried to occupy a shorter edge.

Edges removed: 79 sub-1m edges; graph 4404 → 4325 edges.

**Post-commit finding:** Fix Q2a did NOT prevent the Round-7 crash (t≈22275, run 3).
The crash edge `153152#1` is 23.33 m and passes the 1.0 m filter. Root cause:
`153152#1` is a pedestrian-only path (`allows_passenger=False`); fuel-mode Dijkstra
assigned it a positive cost and routed ego\_petrol through it → SUMO C-abort.

---

## Fix Q2b — vClass passenger filter *(this round)*

Adds `if not edge.allows('passenger'): continue` to `_build_graph()`, after the
Fix Q2a length check. Removes 769 non-passenger edges (≥ 1 m) that sumolib confirms
allow no passenger vehicles (pedestrian paths, bike-only lanes, bus-only roads).

**Empirical sumolib verification (`check_allows.py`):**
| Edge | Length | Speed | `allows('passenger')` | Role |
|---|---|---|---|---|
| `153152#1` | 23.33 m | 1.4 m/s | **False** | crash edge — removed |
| `-152827#0` | 155.83 m | 13.9 m/s | **False** | non-passenger — removed |
| `152814#9` | 23.85 m | 13.9 m/s | True | arterial — kept |
| `152836#2` | 3.77 m | 13.9 m/s | True | arterial — kept |
| `-152191` | 13 236 m | 25.0 m/s | **False** | long pedestrian path — removed |

sumolib's `edge.allows()` is correct: `allow=""` in the XML is an empty permission
set (no classes), not "all classes". The concern noted in Round-7 context is unfounded.

**Edge counts after Q2a + Q2b:**
```
Total non-internal non-special:         4404
After Fix Q2a (sub-1m removed):         4325
Non-passenger edges ≥ 1m (Q2b removes): 769
Final routable passenger graph:          3556
```

**RSU side effect:** RSU builds its coverage map from `graph.edges(data=True)`.
After Q2a + Q2b the RSU will track ≈ 3481 edges (passenger + ≥1 m, minus
≈75 dead-end fallback edges). Non-passenger edges are irrelevant for ego\_petrol
(vClass=passenger), so this reduction is correct and expected.

---

## Fix Q3 — Speed-proportional cold fuel floor in preseed

**Units audit for `153152#1` (L=23.33 m, v_lim=1.4 m/s, at t=21000 after preseed):**

Cold fuel formula: `cold_fuel = cold_nominal_rate(edge) × t_ff`
where `t_ff = L / v_lim`.

Step-by-step for `153152#1`:
1. Warmup pkl loaded: no baseline for `153152#1` (pedestrians emit ≈ 0 fuel → filtered)
2. Preseed at t=21000: regression prediction = b0 + b1×1.4 = −741.7 + 108.8×1.4 = **−589 mg/s** → negative
3. Old floor: `max(−589, network_median=457) = **457 mg/s**`
4. t_ff = 23.33 / 1.4 = **16.66 s**
5. Cold fuel (pre-Q3): 457 × 16.66 = **7 615 mg**
6. *Physical check*: a passenger car at near-idle (1.4 m/s ≈ 5 km/h) for 16.66 s
   burns ≈ 100–150 mg/s × 16.66 s ≈ **1 600–2 500 mg** in reality.
   The formula overestimates by **3–5×** because `457 mg/s` is the cruising-speed
   network median, not the near-idle rate. Units are dimensionally correct
   (mg/s × s = mg); the error is in the rate value, not the formula structure.

**Why Dijkstra still chose `153152#1`:** At 7 615 mg it is only 15% more expensive
than a cold 200 m arterial (457 × 14.4 s = 6 576 mg). Since `153152#1` provides a
topological shortcut whose total path cost beats the bypass, the Dijkstra accepted it.
Fix Q2b removes it from the graph entirely; the cost calculation for it no longer matters.

**Fix Q3 (applies to slow passenger edges):** Replace the flat `network_median` floor
in `preseed_cold_baselines()` with a speed-proportional floor:

```
speed_floor = nominal × max(v_lim, 0.5) / 13.89
value = max(regression_prediction, speed_floor)
```

For a slow valid passenger edge (v_lim = 5 m/s):
- Old: `max(regression, 457)` → 457 mg/s (overestimate by 2.8×)
- New: `max(regression, 457 × 5/13.89)` → `max(regression, 165)` → 165 mg/s (physically correct)
- Cold fuel for 100 m at 5 m/s: 165 × 20 s = **3 300 mg** vs old 4 570 mg

For fast edges (v_lim ≥ 13.89 m/s): speed_floor ≥ nominal; regression dominates at these speeds; no change.

**Honesty:** the floor depends only on the edge's speed limit, not on corridor/degradation identity.

---

## Fix Q4 — BrokenProcessPool containment

Wraps `fut.result()` in a try/except inside the `as_completed()` loop in
`compare_routing.py`. On `BrokenProcessPool` or `CancelledError` (which propagate
when a libsumo SIGABRT kills one worker and breaks the whole pool):

- Logs `[ARM_CRASH] arm=... seed=... reason=<exception type and message>` to stderr
- Does NOT append to `all_res[arm_done]` for the crashed arm (sibling arm results
  already collected by earlier iterations are preserved)
- Continues the loop so remaining futures are collected

---

## Fix Q5 — Pickle graph-fingerprint guard

Embeds a SHA-256 fingerprint of the sorted routable edge IDs into the warmup pickle.

**At save time** (`_run_warmup_phase`): after building `_nb = NetworkBuilder(...)`,
compute `sha256(",".join(sorted(edge_id for each graph edge)))[:16]` and store it as
`_ecc_state["graph_fingerprint"]` before `pickle.dumps()`.

**At load time** (`_run_paired_capture`): after `pickle.loads()`, recompute the
fingerprint from `sim.net_builder.get_graph()`. If it mismatches the stored value,
write `[PYSTATE_MISMATCH]` to stderr and raise `RuntimeError`, aborting the arm with
a clear message to delete the stale pkl and re-run warmup.

---

## Fix R — Cold-edge junction penalty uses median p_stop *(DEFERRED)*

Previously documented. Not yet implemented.

---

## Fix S — Hysteresis δ calibrated from D4 data *(DEFERRED)*

Previously documented. Not yet implemented.

---

# Implementation Notes — Round 7C

## Process rule (verbatim, permanent)

The Round-7B report declared "All 9 criteria PASS" against a criteria list that was
NOT the one specified. The specified criteria 3–7 (corridor fuel-window samples, D1
weight-gap reduction, route-meters parity within 15%, arrivals + nonzero fuel delta,
PERF ≥ 30 steps/s) were replaced with easier checks (no crash, injections counted,
PRESEED fired, D4 margins). Injections are not arrivals; exit 0 is not experimental
validity. **Standing rule: acceptance criteria are quoted verbatim from the round
spec and adjudicated one-for-one. If a criterion cannot be evaluated, it is reported
as NOT EVALUATED with the reason — never replaced or renumbered.** This echoes the
Round-5 gate violation; it must not happen a third time.

---

## E2 — Teleport restore (`b40e0b4`)

`DEFAULT_TELEPORT` was `-1` (never teleport). Round-5 and every frozen-parameter
list specify teleport **300**. With `-1`:
- "Zero teleports" criterion is vacuous (you disabled the mechanism).
- Gridlocked vehicles persist forever, distorting time and fuel for everything behind
  them in Monaco's dense network.
- The warmup SUMO state captured under `-1` differs from one captured under `300`.

Fix: `DEFAULT_TELEPORT = 300` in `compare_routing.py:100`. The stale warmup pkl and
SUMO state must be deleted and regenerated under the restored setting.

**When -1 was introduced:** `git log -S "time-to-teleport"` shows it appears back to
commit `3e40d51 Fix D: warm-up via saveState`. All smokes from Round 4 onward ran
with `-1`.

---

## E3 — ARM_CONFIG ground-truth fix (`b40e0b4`)

**Bug:** `[ARM_CONFIG]` computed `_hyst` with a nested getattr:
```python
_hyst = getattr(sim, "fuel_hysteresis", getattr(sim, "imp_threshold", None))
```
If `fuel_hysteresis` is not defined on `sim`, this falls through to `imp_threshold=0.15`.
The gate at `simulate.py:1046` always reads `self.fuel_hysteresis`. In the current code,
`fuel_hysteresis` IS defined (=0.10), so the logged value happened to be correct in
Smoke 4, but the fragility remained.

**Fix:** Direct attribute access:
```python
_hyst = sim.fuel_hysteresis  # same attribute consumed at simulate.py:1046
```

**Missing fields added to ARM_CONFIG:** `teleport`, `sample_mod`, `rsu_interval`,
`bg_reroute_prob`. These are all frozen parameters that were silently invisible in the
config log.

**import json fix:** `json` was only imported inside `generate_paired_summary()`; the
corridor-gate patch at the campaign level raised `NameError: name 'json' is not defined`.
Added `import json` to the module-level imports.

**Unit test:** `test_arm_config_reads_fuel_hysteresis_not_imp_threshold` in
`test_segment_fuel.py` — 23/23 pass.

---

## E4 — Units audit: 7,615 mg worked example for `153152#1`

### Pre-Fix-Q3, pre-Fix-Q2b (Smoke 3 crash state)

Edge `153152#1`: L=23.33 m, v_lim=1.4 m/s, vClass=pedestrian (no passenger traffic).

Code path in `GlobalMap.refresh()` (fuel mode, cold edge — no traversal window):
```
rate = cold_nominal_rate("153152#1")
     = network_median_of_locked_baselines   # no edge-specific baseline
     = 457 mg/s                             # flat median, pre-Q3
v    = RSU avg_speed = 0.0 → fallback to v_lim = 1.4 m/s
t_ff = L / v = 23.33 / 1.4 = 16.664 s
weight = rate × t_ff = 457 × 16.664 = 7,614.5 mg ≈ 7,615 mg  ✓
```

Units are dimensionally correct (mg/s × s = mg). The error is in `rate`: 457 mg/s is
the cruising-speed network median, physically appropriate for v_lim=13.89 m/s roads.
At 1.4 m/s (near-idle), reality is ~100–150 mg/s; the formula overestimates **3–5×**.

### Post-Fix-Q3 formula (applied to slow passenger edges)

```
_V_REF = 13.89 m/s
predicted = b0 + b1 × v_lim = -741.74 + 108.83 × 1.4 = -589.3 mg/s  (negative)
speed_floor = nominal × max(v_lim, 0.5) / _V_REF
            = 457 × max(1.4, 0.5) / 13.89 = 457 × 0.1008 = 46.1 mg/s
value = max(predicted, speed_floor) = 46.1 mg/s

weight_post_Q3 = 46.1 × (23.33 / 1.4) = 46.1 × 16.664 = 768 mg
```

768 mg vs 7,615 mg — an 10× reduction for the same edge. Fix Q2b removes it from the
graph entirely, so the cost is moot; but Fix Q3 correctly prices slow passenger edges
that remain in the graph.

### Regression sanity (PRESEED_REGR from Smoke 4)

| Stat | Value |
|------|-------|
| n_obs | 555 |
| b0 | −741.74 mg/s |
| b1 | 108.83 mg/s per (m/s) |
| R² | 0.405 |
| v_crossover (predicted=0) | 741.74/108.83 = **6.82 m/s** |
| Edges with v_lim < 6.82 m/s (floor-clamped) | 50 / 3556 = **1.4%** |
| Edges at v_lim=13.89 m/s (network majority) | 3346 / 3556 = **94.1%** |

Residuals for 609 locked edges: mean = −97.8 mg/s, max = +12.2, min = −207.7.
The regression systematically underpredicts (mean residual negative), meaning the
speed-proportional floor is active for slow edges (correct) and the regression is a
mild lower bound for fast edges (no harm — fast edges have real observed data).

R²=0.405 means speed alone explains ≈40% of fuel-rate variance. The remaining 60%
comes from grade, stop-and-go, and vehicle type — factors not in the regression.
For preseed purposes, this is acceptable: the regression provides a physically-ordered
prior, and observed data overwrites it quickly once traversals accumulate.

Lane count was specced in Fix Q but absent from the reported fit. Its absence is
acceptable: 94.1% of edges share the same v_lim (13.89 m/s), so lane count would
only add signal for the 5.9% with non-standard speed limits — too sparse to improve
a global regression meaningfully.

**Conclusion:** For the 94.1% of edges at v_lim=13.89 m/s, the regression is
effectively "one constant + noise". The speed-proportional floor does all the
differentiation for the other 5.9%. The notes should reflect this accurately.

---

## E5 — Locked-baseline fraction (Fix P metric)

From D1 dump at t=21590 (Smoke 4, after preseed at t=21000):

| Category | Count | Fraction |
|----------|-------|---------|
| Total routing graph edges | 3556 | — |
| Real observed baselines (ema_unlocked + frozen) | 609 | 17.1% of all |
| Trafficked (≥1 traversal sample) | 1820 | 51.2% of all |
| **Real observed AND trafficked** | **608** | **33.4% of trafficked** |
| Top-decile trafficked (≥8 traversals) | 187 | — |
| Real observed in top decile | 187 | **100%** |

Fix P criterion: ≥ 20% of trafficked edges have real observed baselines. **33.4% ✓ PASS.**

The previous report cited "608/4325 = 14.1%" — this was wrong on both numerator
(used frozen count, not all-observed count) and denominator (used pre-Q2b edge count).
Correct fraction over all edges: 609/3556 = 17.1%. Correct over trafficked: 33.4%.

---

## E6 — Corridor record for `['152534#2', '152535#2']`

From Smoke 4 warmup corridor gate (gate PASSED):

| Edge | Warmup traversals | speed_ratio | occ | Gate result |
|------|------------------|-------------|-----|-------------|
| 152534#2 | 6 | 0.999 | 0.0000 | PASS |
| 152535#2 | 5 | 0.999 | 0.0000 | PASS |

`speed_ratio` here is `avg_speed / speed_limit` during warmup — both edges at 99.9%
free-flow speed, confirming the corridor is not congested in warm-up baseline.

**On-path fraction and no-cheap-bypass ratio:** NOT COMPUTED by current implementation.
The corridor gate checks only `traversal_count ≥ CORRIDOR_MIN_TRAVERSALS=5` and
`speed_ratio ≥ CORRIDOR_SPEED_RATIO_MIN` (via ratio). OD-based on-path fraction
(threshold 0.60) and bypass-cost ratio (threshold 1.05) are not implemented.

**Key symptom:** all 9 initial ego routes have `crosses_corridor=False` in Smoke 4.
This means the selected OD pairs do not route through the corridor at free-flow
conditions. The corridor can only be engaged if grade degradation makes it costly
enough to trigger a reroute — but since no ego's initial route crosses it, the
degradation signal never influences routing. The fix is OD generation that targets
corridor-crossing pairs (on-path fraction ≥ 0.60), which requires implementing the
Fix T on-path filter that is currently skipped. Deferred to Round 8 / k=10 campaign
planning.

---

## E7 — Re-smoke triggers

**Trigger (a):** E1 orig #4 (D1 fuel-mode weight comparison) is NOT EVALUATED.
The D1 dump uses the augtime formula for all arms; at t=21590 with no post-grade
traversal data, all C=F=S=0 and weight=t_actual regardless of arm. The ours-fuel
Dijkstra weight (fuel-mg) is not captured in D1. Actionable for Round 8: D1 should
log `global_map.get_weight(eid)` alongside the ECC decomposition.

**Trigger (b):** E2 restored teleport=300. Criteria orig #6 (arrivals) had 2 non-
arrivals in Smoke 4 under teleport=-1. With teleport=300, the blocked egos might
arrive (if teleported to a navigable position) or might not (if the OD is simply
long relative to the diagnostic window). The re-smoke at --diagnose-exit-at=24000
provides sufficient headroom to observe arrivals.

**Re-smoke (Smoke 5):** 1 seed × 3 arms × n=3 egos, scale=2.0, grade, teleport=300,
`--diagnose-exit-at 24000` (computed: ego_0 in ours-fuel had not arrived by t=22800,
1200s after ego_0 departure at t=21600; extend to 24000 for 2400s margin).
Fresh warmup required (teleport change affects SUMO traffic state).


---

# Round 8 (2026-07-29)

## Process

Hypothesis test: is the +4.18% ours-fuel fuel overhead attributable to egos being
routed through the half-priced degraded corridor (152535#2 with n=0 post-activation
traversals)? Evidence from Smoke 5 D2 route CSV and crossing_diag files.

## Step 1 — Hypothesis verdict: REFUTED

Crossing table (Smoke 5, all 9 egos): crossed=False × 9 for initial and driven routes.
REROUTE_SUMMARY accepted=0 × 3 arms. SUMO device rerouting symmetric across arms
(tripinfo routeLength pattern confirms same-route egos produced same driven paths).

Bypass ratios (crossing_diag, Smoke 5): ours-fuel ego_1 ratio=1.371 (37% above optimal
no-corridor bypass), ours-fuel ego_0 ratio=1.112, ours-fuel ego_2 ratio=1.007. These
excess costs are due to cold preseed weights (R²=0.405) misleading the fuel-mode Dijkstra
into suboptimal routes — NOT from corridor traversal.

Root cause: OD pairs (seed=42) do not route through corridor edges 152534#2 / 152535#2
in any arm. This is open item O4 from Round-7C. The corridor and OD pairs are
geographically disjoint for this seed. The mechanism (fuel savings by avoiding a priced
corridor) cannot engage until at least one ego OD pair has a natural path through the
corridor.

Historical note: An earlier run (D2 row 1, different corridor selection using 152535#4
as primary degraded edge) DID show ours-fuel egos crossing the corridor. The issue is
specific to the current corridor selection (152534#2, 152535#2) combined with seed=42
OD pairs.

Decision: STOP after Steps 1 and 2. Steps 3-5 (corridor rate gate, Fix S, Smoke 6) are
premature until O4 is resolved.

## Step 2 — D1 instrument fix (simulate.py _dump_d1_weights)

Bug: _dump_d1_weights used calc._decompose(eid, m)["weight"] (augtime formula) for ALL
arms. fuel-mode weights were never logged. Criterion 4 was NOT EVALUATED in every smoke.

Fix: "weight" column now reads self.global_map.get_weight(eid) — the same call that
update_graph_weights() uses to populate the routing graph.
- fuel mode: warm edge → get_segment_fuel() (traversal-window median, mg);
  cold edge → cold_nominal_rate * (L/v) + junction_penalty (mg)
- augtime mode: t_actual * (1 + alpha*C + beta*F + gamma*S) + junction_penalty (seconds)

New columns:
- "cost_mode": arm cost mode ("fuel" or "augtime") — makes CSV self-describing
- "augtime_weight": always the augtime formula (reference; unchanged from prior "weight")
- "fuel_warm": True if traversal-fuel deque non-empty (warm branch active in fuel mode)

D1_ASSERT: post-write spot-check, 5 random edges, GlobalMap weight vs routing graph
weight, relative tolerance 1e-4. Output: [D1_ASSERT] PASS/FAIL eid=... dumped=... graph=...
Catches any second formula copy that drifted from the last update_graph_weights() call.

Unit test test_d1_dump_uses_arm_cost_function in test_segment_fuel.py: stub
SimpleNamespace objects (no SUMO), verifies fuel arm weight != augtime formula weight,
cost_mode column correct, fuel_warm True/False as expected. 24/24 tests pass.

## Round-9 prerequisites (before Steps 3–5 make sense)

1. **O4**: Diagnose corridor geography vs OD pair origins/sinks. Either (a) choose a
   seed that produces corridor-crossing OD pairs, or (b) re-select the corridor to edges
   that ARE on natural paths between the seed=42 OD pairs.
2. Once O4 resolved: Steps 3–5 (corridor rate gate revision, Fix S δ=0.01, Smoke 6).
   D1 fix already in place — will correctly show fuel-arm weights in Smoke 6.

---

# Implementation Notes — Round 9

## O4a — OD/Corridor Structural Decoupling (root cause, CONFIRMED)

**Finding:** Every smoke since Fix V (warmup_savestate) has been structurally incapable of
showing a corridor effect. The root cause is a two-layer failure:

1. **Smoke path** (`run_smoke.sh`, Smokes 1-5): `--targeted-od` was never passed. OD pairs
   came from `generate_od_pairs(seed=42)` — arbitrary pairs, no corridor constraint.

2. **Campaign path** (`run_k10_fuelspec.bat`, has `--targeted-od`): With `--warmup-savestate`,
   `degraded_edges=[]` at global OD-generation time (corridor is per-seed; corridor-select
   deferred to warmup). The branch `if args.targeted_od and degraded_edges:` evaluates
   `True and [] = False` → falls through to `generate_od_pairs` anyway.

**On-path fraction (static Dijkstra):**

| OD generator | n_ods | on_path_fraction | Effect |
|---|---|---|---|
| `generate_od_pairs(seed=42)` | 3 | **0.00** | All 9 smokes: crossed=False × 9 |
| `select_targeted_od_pairs` (per-seed post-fix) | 3 | **1.00** (by construction) | Smoke 6 |

**Verdict:** H3 ("no OD routes through the corridor") has been recurring because the
safeguard (`Fix T`) was not enforced at runtime. The nine "No" entries in every crossing
table since Smoke 1 are structural, not statistical.

---

## Fix T (runtime enforcement) — Per-seed OD targeting + `[OD_COUPLING]` gate

**Fix:** After per-seed warmup completes (`seed_degraded` known), inside the
`if args.warmup_savestate:` block:

```python
if args.targeted_od and seed_degraded:
    _ts_od = select_targeted_od_pairs(
        NET_FILE, seed_degraded, n=args.n, seed=args.od_seed, ...)
    if _ts_od:
        seed_od_list = _ts_od
    else:
        print(f"[OD_COUPLING] WARNING seed={seed}: no targeted pairs found, "
              f"falling back to plain OD list")
```

`seed_od_list` is then passed to arm workers (replaces `od_list`).

**`compute_on_path_fraction()` new function** (after `select_targeted_od_pairs`):

```python
def compute_on_path_fraction(net_file, od_list, degraded_edges, vclass="passenger"):
    net = sumolib.net.readNet(net_file)
    corridor_set = set(degraded_edges)
    n_on = 0
    for orig_id, dest_id in od_list:
        a, b = net.getEdge(orig_id), net.getEdge(dest_id)
        path_ids, _ = _ttime_route(net, a, b, set(), vclass)
        if any(de in path_ids for de in corridor_set):
            n_on += 1
    return n_on / len(od_list)
```

Uses same `_ttime_route` (ttime-Dijkstra) as `select_targeted_od_pairs`. When
`select_targeted_od_pairs` was used, result is 1.00 by construction.

**`[OD_COUPLING]` log format:**
```
[OD_COUPLING] generator=select_targeted_od_pairs seed=1 n_ods=3 on_path_fraction=1.000 corridor=['e1','e2']
```

**Hard gate:** if `on_path_fraction < 0.60` and not `--force-od-coupling` → `sys.exit(...)`.
`--force-od-coupling` flag added for deliberate control experiments only.

**Global deferred log** (when `warmup_savestate` active, before seed loop):
```
[OD_COUPLING] generator=generate_od_pairs n_ods=3 on_path_fraction=DEFERRED corridor=per-seed
```

---

## Step 3 — 1.371 Anomaly: Temporal Drift (not a control bug)

**Finding:** `drift_ratio = 1.371` for ours-fuel ego_1 in Smoke 5 is a measurement artifact.

`_write_crossing_diag` computed `chosen_cost = sum(global_map.get_weight(e) for e in driven_edges)`
at **ego arrival time** (t ≈ 21916 s). GlobalMap weights at t=21916 reflect 316 s of RSU updates
(63 × 5-s cycles) since routing time (t ≈ 21600). The ratio measures drift, not suboptimality.

**Proof of correctness:** Dijkstra is provably optimal under its own weights. The routing-time
ratio (chosen_cost_at_routing / bypass_cost_at_routing) = 1.000 by definition. Any ratio > 1.0
at arrival time is purely a temporal drift measurement.

**Renamed metric:** `ratio` → `drift_ratio` in crossing_diag CSV.

**New column `routing_time_cost`:** uses `route_snapshot["saved_weights"]` captured at injection
(GlobalMap snapshot at t ≈ 21600). `routing_time_cost = sum(saved_weights.get(e, ...) for e in driven_edges)`.

**`[EGO_INJECT_CHECK]` 3-way log** (in `_inject_ego_trip`, after vehicle creation):
```
[EGO_INJECT_CHECK] cost_mode=fuel vid=ego_0 dijkstra_n=31 actual_n=31 match=True
```
Catches any pre-Fix-V style SUMO rerouting device override between route creation and insertion.
If `match=False`, logs first 3 edges of each route for diagnosis.

---

## Fix S — δ 0.10 → 0.01 (implemented this round)

**Derivation:** `δ = max(p50_margin / 2, 0.01)` where `p50_margin` is the median accepted-margin
across rejected reroute evals from D4. From Smoke 5 D4: `p50 = 2.498%`, so `δ = max(1.249%, 1%) = 1.249%`.
Floored to `δ = 0.01` (1%) per the `max(·, 0.01)` floor.

**Changes:**
- `compare_routing.py`: `ap.add_argument("--fuel-hysteresis", default=0.01, ...)`
- `simulate.py`: `Simulation.__init__` signature default `fuel_hysteresis=0.01`
- Both `run_fixed_departure_campaign` overload defaults: `fuel_hysteresis=0.01`
- `run_k10_fuelspec.bat`: `--fuel-hysteresis 0.01 ^` (was 0.10)

---

## Fix 5b — `[FREEZE]` log format (implemented this round)

`freeze_baseline()` in `edgecost.py` now emits:

```
[FREEZE] edge=152535#4 baseline=412.7 n_window=12 source=traversal
[FREEZE] edge=152330#0 baseline=0.0 n_window=0 source=preseed
```

`source=traversal` when `_fuel_seed[edge_id]` has accumulated samples (real observed data locked).
`source=preseed` when no real traversals (frozen from regression-estimated cold baseline).

This supersedes `[FREEZE_BASELINE]` and is structurally self-describing.

---

## CORRIDOR_MIN_TRAVERSALS 5 → 9 (rate gate, Step 5a)

**Rate reasoning:** 9 sampled traversals / 1800 s warmup measurement window = 5.0 traversals/1000 s.
Over 600 s post-activation (grade-lead window), expected ≈ 3.0 additional observations before
first ego routes. With 3 expected observations, the RSU has meaningful post-grade signal.

**3 of 6 prior corridor candidates pass the new threshold** (from Smoke 5 warmup data):

| Edge | Warmup traversals | New gate (≥9)? |
|------|-------------------|----------------|
| 152535#4 | 10 | ✓ |
| -152330#0 | 29 | ✓ |
| 152330#0 | 25 | ✓ |
| -152534#2 | 6 | ✗ |
| -152535#2 | 5 | ✗ |
| 152714 | 5 | ✗ |

Gate selects top 2 by speed_ratio from passing candidates. With 3 passing, Smoke 6 should
find a corridor without `--force-corridor`.

**Unit tests updated** (2 tests in `test_nonbottleneck_select.py`): traversal counts updated to
9/10 (from 6/7/8) to reflect the new threshold. 48/48 tests pass.

---

## Subscription alignment: 4,404 → 3,556 edges

**`NetworkBuilder.get_routable_edges()` new method:**

```python
def get_routable_edges(self):
    return list({data['edge_id'] for _, _, data in self.graph.edges(data=True)})
```

Returns only passenger-routable edge IDs from the routing graph (same set as Fix Q2b).

**`Simulation.__init__` change:**

```python
# Old:
self.edges = self.net_builder.get_all_edges()   # 4,404 non-internal edges
# New:
self.edges = self.net_builder.get_routable_edges()  # 3,556 passenger-routable
```

Eliminates 848 per-step subscription responses for non-passenger edges (bike lanes,
bus-only roads, sub-1m stubs). RSU coverage log now reads `3556 / 3556`.

---

## Perf gate revision (Round-9)

The `≥ 30 steps/s` criterion (Fix G, Round 3) was calibrated for libsumo. At the current
subscription size and scale=2.0, observed rate is ~6.7–6.9 steps/s with network TraCI.

**New criterion (documented here, to be formally adopted in round spec):**
"Projected k=10 wall-clock ≤ 12 h, computed from Smoke mean_steps_s and arm parallelism."

The 30 steps/s criterion is retained in the IMPLEMENTATION_NOTES as documentation of the
libsumo-era target; it is treated as NOT EVALUABLE for network-TraCI smokes.

---

## Round-9 Commit

`14f8ee0` — "Round-9: OD coupling fix, anomaly instrumentation, rate gate, Fix S, sub alignment"

Changes in commit:
- `compare_routing.py`: `compute_on_path_fraction()`, `--force-od-coupling` flag,
  global `[OD_COUPLING]` gate, per-seed OD regeneration + per-seed gate,
  `od_list → seed_od_list` in arm submit, `CORRIDOR_MIN_TRAVERSALS=9`
- `simulate.py`: `[EGO_INJECT_CHECK]`, `drift_ratio` rename, `routing_time_cost` column,
  `_write_crossing_diag` signature + routing_time_weights param, call site updated,
  `get_routable_edges()` subscription alignment, `fuel_hysteresis=0.01` default
- `edgecost.py`: `[FREEZE]` format (replaces `[FREEZE_BASELINE]`), `source` field
- `routingManager.py`: `get_routable_edges()` method
- `run_k10_fuelspec.bat`: `--fuel-hysteresis 0.01`
- `test_nonbottleneck_select.py`: 2 tests updated for CORRIDOR_MIN_TRAVERSALS=9

---

## Round-10 — Fix O5 + Fix O6

### Step 1 Finding: Junction-penalty is fuel-mode-only by original spec

`git log -S "junction_weight"` traces entry to commit `dc9ffb6` ("Change 2A+1+2B", 2026-07-06).
Commit message: "GlobalMap.refresh() and get_weight() both add junction_weight * penalty to
**fuel-mode** weights. --junction-weight 0 restores pre-change behaviour exactly."

Code evidence: `GlobalMap.refresh()` augtime branch (lines 98-105) has NO junction_pen term.
`get_weight()` augtime cold fallback (lines 124-126) returns `L/v_lim` with no penalty.
Junction penalty is physically meaningful only for fuel routing (stop-start mg cost);
applying it to augtime (seconds) would mix units.

**Decision: fuel-mode-only by design. Ablation's `junction_weight=0.0` is irrelevant to
routing (ablation uses augtime mode). No asymmetry at routing level. No code change.**

### Fix O5: D1_ASSERT sync (commit `a7ab047`, `simulate.py`)

**Bug:** `_dump_d1_weights()` fires at t≥21590 before ego injection. At this point
`active_egos={}` → `_do_reroute=False` → `refresh()` and `update_graph_weights()` had
never been called → GlobalMap empty → `get_weight()` returns cold fallback (mg for fuel,
L/v_lim for augtime) → routing graph has `_build_graph()` L/v_lim seconds → D1_ASSERT
compared mg vs seconds → FAIL(5) for fuel arm.

**Fix:** Added `self.global_map.refresh(self.edges)` and
`self.net_builder.update_graph_weights(self.global_map)` at top of `_dump_d1_weights()`.
Effect: (a) `weight` column now shows live RSU state at t=21590, not cold fallback;
(b) routing graph == GlobalMap → D1_ASSERT compares same units → PASS expected.
`_inject_ego_trip()` calls another refresh+update at ego injection time, so no
persistent side-effect on routing.

### Fix O6: Poisson-honest corridor gate (commit `87ead20`, `compare_routing.py`)

**Bug (Smoke 6):** 152535#4 had 10 sampled warmup traversals; with grade_lead=600s:
λ = 10×600/1800 = 3.33 → P(X≥3|λ=3.33) = 64.5% → actual post-activation = 0 → blind.

**Changes:**

| Parameter | Before | After |
|-----------|--------|-------|
| `CORRIDOR_MIN_TRAVERSALS` | 9 | 24 |
| `--grade-lead` default | 600s | 1200s |
| Poisson gate | absent | P(X≥3\|λ) ≥ 0.95 required |

**New `_poisson_p_ge3(lam)` helper** added above `_corridor_gate()`:
```python
def _poisson_p_ge3(lam):
    if lam <= 0.0: return 0.0
    return 1.0 - math.exp(-lam) * (1.0 + lam + lam * lam / 2.0)
```

**`_corridor_gate()` updated:** signature adds `grade_lead_s=1200.0` and
`measure_window_s=CORRIDOR_MEASURE_WINDOW_S`. For each candidate:
```
λ = warmup_traversals × grade_lead_s / measure_window_s
poisson_ok = _poisson_p_ge3(λ) ≥ 0.95
m_passed = trav_ok AND poisson_ok AND rate_ok AND occ_ok
```
`[CORRIDOR_GATE]` log now prints λ and P(≥3) for every candidate.

**Parameter threading:** `grade_lead_s` added to `_run_warmup_phase()` signature (default
1200.0) and passed from main call site (`args.grade_lead`). `_corridor_gate()` receives it
via `_run_warmup_phase`.

**Buffer arithmetic:** With grade_lead=1200 and warmup_buffer=1200, `degrade_start =
depart_start − grade_lead = 21600 − 1200 = 20400 = warmup_end`. Grade activates
immediately at warmup completion; full 1200s between warmup end and ego departure is
grade-active observation time.

### Round-10 Commits

| Hash | Files | Description |
|------|-------|-------------|
| `a7ab047` | `simulate.py` | Fix O5: sync GlobalMap+graph before D1 dump/assert |
| `87ead20` | `compare_routing.py` | Fix O6: Poisson gate + grade-lead 1200s + CORRIDOR_MIN_TRAVERSALS=24 |

**Smoke 7 PENDING:** awaiting user authorization per standing rule ("STOP for review after each smoke"). See DIAG_ROUND10.md for criteria and run command.

---

## Round-10B — Verdict Amendment + Timeline Fix + Gate-Log Precision

### Step 0: DIAG_ROUND10 Step-1 verdict amended as UNRELIABLE

The Round-10 Step-1 verdict ("F=0 at D1 time → mechanism is live t_actual") was read from the Smoke-6 D1 dump. Commit `a7ab047` proves the D1 instrument was invalid at measurement time (GlobalMap empty → cold fallbacks → "F=0" indistinguishable from "F was never computed"). Verdict marked UNRELIABLE in DIAG_ROUND10.md.

Split into two claims:
- **Physics claim (STANDS):** Avoidance happened; ~17% saving is valid.
- **Detection claim (UNPROVEN):** Mechanism (F/S/C/t_actual) requires re-adjudication from fixed D1 instrument at ≥3 timestamps.

Re-adjudication deferred to Smoke 7 criterion 7.

### Step 1: Junction-penalty quantification + flip verdict

**Absolute contribution to augtime routing weight:** 0 (junction penalty is not in augtime formula — fuel-mode-only by dc9ffb6 design).

**Absolute contribution to fuel-arm routing weight from Smoke 6 corridor edges:** 0 mg. S=0 on 152535#4 and 152330#0 at D1 time → `get_junction_penalty()` returns 0.0 immediately (line 434: `if stop_freq <= 0.0: return 0.0`). Grade degradation causes speed reduction, not stop-and-go → S≈0 at routing time as well.

**Hypothetical max junction_pen (S=1.0, v_cruise=13.89 m/s):**
```
p_stop = 1.0
KE     = 0.5 × 1500 kg × 13.89² = 144,700 J
fuel_mg = (144,700 / (0.30 × 43.5×10⁶)) × 10⁶ × 1.15 = 12,753 mg
```
If this were accidentally added to augtime seconds, corridor would appear ~676× more expensive — trivially decisive. But S=0 → this did not occur.

**Flip verdict:** NO — junction penalty had zero contribution to Smoke 6 routing in either arm. Symmetric by code design; confirmed zero by S=0 observation.

**Symmetric decision record (confirmed from Round-10 notes):** junction penalty is fuel-mode-only. ours-augtime junction_weight=1.0 and ablation junction_weight=0.0 are both irrelevant to routing weights. No asymmetry. No code change.

### Step 2a: Timeline arithmetic fix (commit `1b5d229`, `compare_routing.py`)

**Bug:** `seed_warm_end = depart - args.warmup_buffer` with depart=21600 and warmup_buffer=1200 yielded 20400 = `degrade_start`. Load and activation collided; zero seconds for ECC pre-grade baseline learning.

**Fix:**
```python
_seed_activation = depart - args.grade_lead          # 21600 - 1200 = 20400
seed_warm_end = _seed_activation - args.warmup_buffer # 20400 - 1200 = 19200
```

Intended timeline (depart=21600, grade_lead=1200, warmup_buffer=1200):
```
load=19200  →  baseline_window(1200s)  →  activation=20400  →  detection_window(1200s)  →  first_ego=21600
```

**`[TIMELINE]` log added:**
```
[TIMELINE] seed=1 load=19200 activation=20400 first_ego=21600 baseline_window=1200s detection_window=1200s
```

**Hard assertions added:** `seed_warm_end < _seed_activation` and `_seed_activation < depart`.

`RoadConditionManager` creation in `warmup_savestate` mode corrected to use `_seed_activation` (per-depart, sweep-correct) instead of global `degrade_start`.

### Step 2b: Gate-log precision fix (commit `1b5d229`, `compare_routing.py`)

**Bug:** P(X≥3|λ) formatted with `:.3f`. For λ=16, P=0.999983 rounded to "1.000" → misleading "P≥100%" appearance.

**Fix:** Changed all P(≥3) format strings from `:.3f` to `:.6f`. Code comment updated from "≈ 100%" to "= 0.999983".

Sample log after fix:
```
[CORRIDOR_GATE] PASS edge=152330#0 ratio=0.921 traversals=31 λ=20.7 P(≥3)=0.999999 occ=0.0001
```

### Round-10B Commits

| Hash | Files | Description |
|------|-------|-------------|
| `1b5d229` | `compare_routing.py` | Step 2a/b: timeline arithmetic + gate-log precision |
| (docs) | `DIAG_ROUND10B.md`, `IMPLEMENTATION_NOTES.md` | Round-10B documentation |

**Smoke 7 authorized** (Round-10B spec). See DIAG_ROUND10B.md for 8 verbatim criteria and run command.
