# DIAG_ROUND7C.md — Round-7C / Smoke 5 Re-adjudication

**Date:** 2026-07-26
**Branch:** `fuel-model-v2`
**HEAD at run:** `af5b8b4`
**Smoke:** 5 (re-run after E2+E3 fixes; replaces Smoke 4 for criteria adjudication)

**Command:**
```
conda run -n ml python -u -m Simulation.compare_routing \
  --paired --n 3 --seeds 1 --scale 2.0 \
  --arms ours-fuel,ours-augtime,ablation \
  --road-condition grade --degraded-edges auto-nonbottleneck \
  --warmup-savestate --diagnose --diagnose-exit-at 24000 \
  --bg-reroute-prob 0.25 --depart-start 21600 \
  --warmup-buffer 1200 --grade-lead 600 \
  --vehicle-sample-mod 2 --rsu-update-interval 5 \
  --fuel-hysteresis 0.10 --force-corridor
```

**Changes from Smoke 4:**
- `--time-to-teleport 300` restored (was -1, unauthorized; see E2 in IMPLEMENTATION_NOTES Round-7C)
- `--diagnose-exit-at 24000` (extended from 22800 to ensure all egos could arrive)
- Stale pkl + `ws_1_200_21600.xml.gz` deleted before run; fresh warmup rebuilt
- `[ARM_CONFIG]` now logs `fuel_hysteresis` directly (not `imp_threshold`), plus `teleport`, `sample_mod`, `rsu_interval`, `bg_reroute_prob` (E3)
- `import json` added at module level; corridor-gate metadata patch no longer raises NameError (E3)

---

## Re-adjudication: 9 Original Criteria (verbatim from round spec)

| # | Criterion (verbatim) | Evidence | Status |
|---|---------------------|----------|--------|
| 1 | No crash / exit 0 | Exit code 0; no `[ARM_CRASH]` in output; all 3 arms terminated normally | **PASS** |
| 2 | 9/9 ego injections (3 per arm) | `[EGO_ROUTE]` × 9; `[ARRIVAL_COUNTS]` ours=3/3, ablation=3/3 | **PASS** |
| 3 | Corridor fuel window ≥3 post-activation samples on 152534#2 AND 152535#2 before first ego | `[FUEL_WINDOW]` at t=21600: 152534#2 n_traversals=3 ✓; 152535#2 n_traversals=0 ✗ | **FAIL** |
| 4 | D1 re-dump comparison: cold vs observed weight-per-meter, fuel and augtime arms vs Round-6 numbers | D1 generated for all 3 arms at t=21590 (3556 edges each). `_dump_d1_weights()` always uses augtime formula (`_decompose()`); at t=21590, C=F=S=0 for all edges (no post-grade-activation traversals yet), so all weights equal t_actual regardless of arm — fuel-mode Dijkstra weights not captured. | **NOT EVALUATED** |
| 5 | EGO_ROUTE meters per ego per arm, within 15% of ablation; route divergence post-activation | ego_0: abl=6421m, fuel=7017m(+9.3%), aug=6794m(+5.8%); ego_1: abl=1547m, fuel=1643m(+6.2%), aug=1547m(0.0%); ego_2: abl=5346m, fuel=5990m(+12.0%), aug=5990m(+12.0%) — all 6 within 15%. No post-activation divergence (crosses_corridor=False × 9). | **PASS** |
| 6 | All 3 egos arrived in all 3 arms? Per-ego fuel totals; paired fuel delta sign and magnitude | 9/9 arrived (3/3 per arm). Per-ego fuel: see table below. Total ego fuel vs ablation: ours-fuel +4.18% (56,190mg MORE); ours-augtime −11.66% (156,653mg LESS). No corridor crossing (crosses_corridor=False × 9) — differences are initial-route artefacts, not grade-penalty savings. | **PASS** (arrivals 9/9; fuel delta nonzero ✓) |
| 7 | PERF steps/s in heavy phase per arm; ≥30 with libsumo + Fix W? | `[PERF_SUMMARY]` ours-fuel=6.7 steps/s, ours-augtime=6.9, ablation=6.9 — all << 30 | **FAIL** |
| 8 | `[VCLASS_ASSERT]` PASS × 4 | warmup + 3 arms: `graph_edges=3556 non_passenger=0` × 4 | **PASS** |
| 9 | `[PYSTATE]` graph_fp match × 3 arms | warmup wrote `db665389287b43ea`; ours-fuel, ours-augtime, ablation all matched | **PASS** |

**Round-7C verdict: 6 PASS · 2 FAIL · 1 NOT EVALUATED**

---

## Smoke 4 → Smoke 5 Delta

| # | Smoke 4 | Smoke 5 | Change |
|---|---------|---------|--------|
| 1 | PASS | PASS | — |
| 2 | PASS | PASS | — |
| 3 | FAIL | FAIL | — |
| 4 | NOT EVALUATED | NOT EVALUATED | — |
| 5 | PASS | PASS | — |
| 6 | PARTIAL (7/9 arrived) | PASS (9/9 arrived) | ✓ improved |
| 7 | FAIL | FAIL | — |
| 8 | PASS | PASS | — |
| 9 | PASS | PASS | — |

The 9/9 improvement (vs 7/9) is attributable to: (a) `teleport=300` resolving background gridlock, giving egos a cleaner traffic state at injection; (b) `--diagnose-exit-at 24000` allowing 1,400s more sim-time for slow-route egos.

---

## ARM_CONFIG Verification (E3 confirmed live)

```
[ARM_CONFIG] arm=ours-fuel     cost_mode=fuel    alpha=1.0 beta=8.0 gamma=10.0 junction_weight=1.0 fuel_hysteresis=0.1 aggregator=median teleport=300 sample_mod=2 rsu_interval=5 bg_reroute_prob=0.25
[ARM_CONFIG] arm=ours-augtime  cost_mode=augtime alpha=1.0 beta=8.0 gamma=10.0 junction_weight=1.0 fuel_hysteresis=0.1 aggregator=median teleport=300 sample_mod=2 rsu_interval=5 bg_reroute_prob=0.25
[ARM_CONFIG] arm=ablation      cost_mode=augtime alpha=0.0 beta=0.0 gamma=0.0  junction_weight=0.0 fuel_hysteresis=0.0 aggregator=median teleport=300 sample_mod=2 rsu_interval=5 bg_reroute_prob=0.25
```

`fuel_hysteresis=0.1` confirmed (previous bug logged `imp_threshold=0.15`). `teleport=300` confirmed restored. No `[WARN] could not patch per_seed_corridor_gate` — `import json` fix working.

---

## Per-Ego Fuel and Route Data

### Initial Route Lengths (EGO_ROUTE at injection)

| Ego | Ablation | Ours-fuel | Δ fuel | Ours-augtime | Δ aug |
|-----|----------|-----------|--------|--------------|-------|
| ego_0 | 6421.0 m | 7017.0 m | +9.3% | 6793.9 m | +5.8% |
| ego_1 | 1547.0 m | 1643.1 m | +6.2% | 1547.0 m | 0.0% |
| ego_2 | 5345.7 m | 5989.5 m | +12.0% | 5989.5 m | +12.0% |

All 6 within 15% threshold. Criterion 5 PASS.

### Per-Ego Fuel Totals (tripinfo.xml)

| Ego | Ours-fuel (mg) | Ours-augtime (mg) | Ablation (mg) |
|-----|---------------|------------------|---------------|
| ego_0 | 617,953 | 450,139 | 408,219 |
| ego_1 | 193,984 | 148,484 | 148,498 |
| ego_2 | 587,622 | 588,093 | 786,652 |
| **Total** | **1,399,559** | **1,186,716** | **1,343,369** |

Paired delta vs ablation:
- **ours-fuel:** +56,190 mg = **+4.18%** (uses more fuel)
- **ours-augtime:** −156,653 mg = **−11.66%** (uses less fuel)

ego_2 note: ablation routed ego_2 on a 25.3% higher-fuel path than ours (786,652mg vs ~588,000mg). This is a pure initial-route divergence — SUMO built-in router chose a slower route for ego_2 while our Dijkstra chose a shorter/faster one. No corridor crossing in any arm.

### Arrival Times

| Ego | Ours-fuel | Ours-augtime | Ablation |
|-----|----------|-------------|---------|
| ego_0 | 22595.5 s | 22329.8 s | 22283.8 s |
| ego_1 | 21916.0 s | 21871.8 s | 21871.8 s |
| ego_2 | 22411.3 s | 22410.8 s | 22659.3 s |

All 9 arrived before exit-at=24000. Arm end times: ours-augtime=22411s, ours-fuel=22659.5s, ablation≈22659.3s (all natural terminations, not forced exits).

---

## Corridor Statistics (Smoke 5)

Gate candidates identical to Smoke 4 (same seed, same warmup state):

| Edge | Traversals (warmup) | Ratio | Gate Result |
|------|---------------------|-------|-------------|
| 152535#4 | 10 | 0.996 | PASS |
| 152534#2 | 6 | 0.999 | PASS |
| 152535#2 | 5 | 0.999 | PASS |
| 152714 | 5 | 0.990 | PASS |
| -152330#0 | 29 | 0.936 | PASS |
| 152330#0 | 25 | 0.977 | PASS |

Selected: `['152534#2', '152535#2']` gate=passed.

Post-activation traversal window (21000→21600, 600s window):
- 152534#2: **n_traversals=3** → ≥3 ✓
- 152535#2: **n_traversals=0** → ≥3 ✗ — criterion 3 FAIL

152535#2 accumulates 5 traversals over the full warmup (14400→20400 = 6000s) but 0 in the 600s post-activation gap before the first ego. The baseline (562.2mg) is frozen from warmup. This pattern held in both Smoke 4 and Smoke 5, indicating a structural gap at this seed/scale, not a teleport-related artifact.

---

## PERF Analysis

Heavy phase = sim_t 21600–22659 (all active egos):

| Arm | mean steps/s | peak active veh | wall time |
|-----|-------------|----------------|-----------|
| ours-augtime | 6.9 | 3 | 1,167 s |
| ours-fuel | 6.7 | 3 | 1,307 s |
| ablation | 6.9 | 3 | 1,309 s |

Threshold ≥30. Actual ≈6.7–6.9. FAIL by factor ~4.4×. Root cause (structural, not seed-dependent): scale=2.0 loads 4,404 RSU edge subscriptions per-step; Fix W (20-step RSU decimation) is active but the subscription set size itself dominates. Unchanged from Round-6 and Smoke 4 analysis.

`[PERF_SUMMARY]` projected k=10: ~3.2–3.6h total.

---

## Teleports

- ours-augtime: 1 teleport, Wrong Lane (background vehicle)
- ours-fuel: 1 teleport, Wrong Lane (background vehicle)
- ablation: 0 teleports

No ego teleports in any arm. Wrong-Lane teleports are SUMO geometry artifacts expected with teleport=300 at scale=2.0.

---

## Reroute Summary

| Arm | evals | accepted | best_rejected_margin |
|-----|-------|----------|----------------------|
| ours-fuel | 61 | 0 | 2.6% |
| ours-augtime | 51 | 0 | 0.0% |
| ablation | 57 | 0 | 0.0% |

Zero accepted reroutes: all ego initial routes have crosses_corridor=False. The grade penalty on 152534#2 / 152535#2 is never encountered during ego traversal, so no fuel-mode saving opportunity arises. Hysteresis gate correctly rejects sub-threshold candidates (max margin 2.6% < 10% threshold).

`[D4_MARGINS]` ours-fuel: n=14, p50=2.498%, p90=2.515%, max=2.636% — all < 10%.

---

## Open Items for Round 8

| ID | Item | Criterion | Priority |
|----|------|-----------|----------|
| O1 | 152535#2 never gets ≥3 post-activation traversals before first ego in 600s window (warmup gate pass requires 5 full-warmup traversals; post-activation window too short). Options: extend warmup buffer, lower ≥3 threshold to ≥1, or re-examine corridor-edge selection criteria. | #3 FAIL | HIGH |
| O2 | D1 weight-gap NOT EVALUATED: `_dump_d1_weights()` uses augtime formula for all arms; fuel-mode Dijkstra weights not exported. Need separate fuel-weight CSV artifact at `GlobalMap.refresh()` time. | #4 NOT EVALUATED | MEDIUM |
| O3 | PERF ≥30 steps/s structurally unachievable at scale=2.0 with 4,404-edge RSU subscriptions. Options: RSU subscription pruning (skip unoccupied edges), or revise threshold to reflect libsumo/scale reality. | #7 FAIL | HIGH |
| O4 | crosses_corridor=False × 9: No ego ever routes through the degraded corridor in 3 OD pairs at seed=1. Without corridor crossing, the grade-penalty savings mechanism cannot engage. Root cause needs investigation (OD pair generation, corridor geographic location vs OD sources/sinks). | Signal gap | HIGH |
| O5 | Fix T (on-path fraction threshold 0.60): not implemented in corridor gate. | E6 gap | MEDIUM |

---

## Warmup Summary (Smoke 5)

- Wall time: 3600.99 s (6 h sim: 14400→20400)
- State: `ws_1_200_21600.xml.gz` 361,510 bytes (rebuilt; prior stale file deleted)
- Python state: `warmstate_seed1_python.pkl` 169,650 bytes
  - sha256=`d0289100195cc978`, graph_fp=`db665389287b43ea`
  - baselines=487, traversal_edges=1592
- RSU coverage: 3556 / 3556 non-internal edges

---

## Commits (Round-7C)

| Hash | Description |
|------|-------------|
| `af5b8b4` | Round-7C IMPLEMENTATION_NOTES: process rule + E2–E7 analysis |
| `b40e0b4` | E2+E3 fixes: teleport=300, ARM_CONFIG fuel_hysteresis, import json, unit test 23/23 |

Prior Round-7B HEAD: `65b8507` (Smoke 4 — criteria evaluated against substituted list, now superseded by this re-adjudication).

---

## Status

**STOP for review.** No k=10 campaign launched.

Per task spec: if route-meters parity (#5 PASS) and arrivals + nonzero fuel delta (#6 PASS) are confirmed by user review, the next session convenes the k=10 discussion. Open items O1, O3, O4 are blocking for meaningful signal at k=10 scale.
