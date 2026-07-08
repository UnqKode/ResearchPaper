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
