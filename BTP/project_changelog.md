# Project Changelog

## Phase 3 — Fuel-specificity campaign (`fuel-model-v2` branch)

### Round 1 (previous session)
Implemented the seven Phase-3 changes from the research plan:
- Change 1: junction penalty in `edgecost.py` + `GlobalMap.refresh` in `simulate.py`
- Change 2A: configurable fuel aggregator (median/mean/ewma) + max-age eviction in `edgecost.py` / `rsuController.py`
- Change 2B: route-switch hysteresis in `simulate.py` + `--use-hysteresis` / `--fuel-hysteresis` CLI flags
- Change 3: `select_nonbottleneck_grade_edges`, three-arm campaign (ours-fuel / ours-augtime / ablation), equal-time decomposition, `run_k10_fuelspec.bat`

### Round 2 — Bug fixes for k=10 campaign launch
| Commit | Fix |
|--------|-----|
| `f8e3ede` | Replaced probe-based edge selector with sumolib-only structural selection (speed limit 7–20 m/s + length ≥ 100 m + OD coverage ≥ 0.20). Eliminated the failed SUMO probe that crashed due to insufficient traffic at 4 AM. |
| `a1f26ae` | Fixed `ego_routing` mapping: arm names `ours-fuel` / `ours-augtime` now correctly map to `ego_routing="ours"`; only `"sumo"` maps to `ego_routing="sumo"`. |
| `d2ed1e9` | Fixed stale TraCI connection on worker process reuse: unconditional `try: traci.close()` before `traci.start()` in `run_paired_scenario`. |

### Round 3 — Performance rescue + corridor validity (this session)
| Commit | Fix |
|--------|-----|
| `Fix A` | libsumo compat shim (`BTP/traci_compat.py`): prefers in-process libsumo (10–50× faster), falls back to network TraCI. All `import traci` replaced with compat import. `traci.start()` guarded for libsumo constraints. |
| `Fix B` | Vehicle sampling (SAMPLE_MOD=5): ~20% of background vehicles subscribed per step, eliminating O(active\_vehicles) subscription payload growth. Ego vehicles always subscribed. `--vehicle-sample-mod` CLI flag (default 5). Edge subscriptions extended with `VAR_FUELCONSUMPTION` + `LAST_STEP_MEAN_SPEED` for corridor gate. |
| `Fix C` | Cached `getDeltaT()` in `subscribe_edges()`; `rsu_manager.step(sim_time)` now accepts sim\_time from caller. Eliminated 2 redundant TraCI socket round-trips per step (~20 ms/step saved on Windows localhost TCP). |
| `Fix D` | Per-seed saveState warmup: one headless SUMO runs 14400 → 21000, saves state; all 3 arms load it at startup. Eliminates 3× redundant warm-up simulation (was the 16-hour bottleneck). |
| `Fix E` | Corridor validity gate: structural selector returns k=6 candidates; phase-0 warmup measures speed\_ratio and occupancy over last 1800 sim-sec under real demand; best 2 passing `ratio≥0.85 AND occ≤0.40` become degraded edges. Restores the free-flow check dropped by `f8e3ede`. `--force-corridor` flag bypasses abort. |
| `Fix F` | Arm-params integrity: `[ARM_CONFIG]` log + assertions on cost\_mode and α/β/γ per arm in `run_paired_scenario`. Unit test `test_arm_params_all_arms` added. |
| `Fix G` | Step-rate telemetry: `[PERF]` line to stderr every 500 heavy-phase steps with steps/s, sim\_t, active\_veh. Acceptance criterion: ≥ 30 steps/s post fast-forward. |

**bat file:** `run_k10_fuelspec.bat` updated with `--seeds 1,2,3,4,5,6,7,8,9,10` (comma-separated), `--warmup-savestate`, `--vehicle-sample-mod 5`.

### Round 4 — Baseline starvation fix (fuel delta = 0% → nonzero)

**Root cause diagnosed in Round-4:** With warmup_buffer=600s and sample_mod=5, low-traffic
degraded edges collected fewer than 2 samples before `freeze_baseline()` fired at grade
activation. `_fuel_baseline` was `None`, so `F=0` on all edges, making ours-fuel/ours-augtime/ablation
identical → 0% delta.

| Fix | Description |
|-----|-------------|
| **Fix 1** | `--warmup-buffer 1200` (default, was 600). warmup ends at `depart_start - 1200 = 20400`, giving RSU 1200 s before grade activation. |
| **Fix 2** | `--vehicle-sample-mod 2` (default, was 5). Doubles the traversal sample rate; baseline fills in ~half the time at minimal step-rate cost. |
| **Fix 3** | `--grade-lead 600` (default, was 300). Grade activates at `depart_start - 600 = 21000`, giving RSU 600 s of free-flow data before egos depart. |
| **Fix 4** | `[BASELINE_GATE]` per edge at activation: `PASS baseline=Xmg` or `FAILED baseline=None`. Raises `RuntimeError` aborting the seed on failure (use `--force-baseline` to demote to warning). `[FUEL_WINDOW]` at first ego departure shows n_traversals / baseline / frozen for each degraded edge. |
| **Fix 5** | End-of-arm `[REROUTE_SUMMARY]` with `reroute_evals=N accepted=M best_rejected_margin=X%`. Per-decision `[FUEL_HYST]` KEEP lines now include `margin=X%`. Per-decision `[DECISION]` KEEP lines include `improvement=X%`. |
| **Fix 6** | End-of-arm `[PERF_SUMMARY]` to stderr: `total_steps`, `mean_steps_s`, `peak_active_veh`, `wall_s`, and projected k=10 wall-clock estimate. |

**bat file:** `run_k10_fuelspec.bat` updated with `--vehicle-sample-mod 2 --warmup-buffer 1200 --grade-lead 600`.
