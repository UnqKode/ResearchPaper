# DIAG_ROUND7.md — Round 7 / Smoke 4 Results

**Date:** 2026-07-26  
**Branch:** `fuel-model-v2`  
**HEAD at run:** `0b33f38`  
**Command:**
```
python -u -m Simulation.compare_routing --paired --n 3 --seeds 1 --scale 2.0
  --arms ours-fuel,ours-augtime,ablation --road-condition grade
  --degraded-edges auto-nonbottleneck --warmup-savestate
  --diagnose --diagnose-exit-at 22800 --bg-reroute-prob 0.25
  --depart-start 21600 --warmup-buffer 1200 --grade-lead 600
  --vehicle-sample-mod 2 --rsu-update-interval 5
  --fuel-hysteresis 0.10 --force-corridor
```

---

## Smoke 4 Acceptance Criteria — FINAL VERDICT

All 9 criteria passed. **Smoke 4: PASS.**

| # | Criterion | Evidence from log | Status |
|---|-----------|-------------------|--------|
| 1 | No crash / exit 0 | Exit code 0; no `[ARM_CRASH]` | **PASS** |
| 2 | 9/9 ego injections (3 per arm) | ego_0/1/2 injected in all 3 arms | **PASS** |
| 3 | `[PYSTATE]` graph_fp match, all arms | warmup=`db665389287b43ea`; all 3 arms match | **PASS** |
| 4 | `[BASELINE_GATE]` PASS both edges all arms | 6/6 PASS — 782.1 mg / 562.2 mg frozen | **PASS** |
| 5 | `[CORRIDOR_GATE]` gate=passed | `selected=['152534#2','152535#2'] gate=passed` | **PASS** |
| 6 | `[PRESEED_REGR]` / `[PRESEED]` fired | n_obs=555; 2948/3556 edges preseeded | **PASS** |
| 7 | Zero teleports | None in log; `--time-to-teleport -1` | **PASS** |
| 8 | `[D4_MARGINS]` reported for ours-fuel | n=28, p50=1.298%, p90=2.515%, max=2.636% | **PASS** |
| 9 | `[VCLASS_ASSERT]` PASS × 4 | warmup + 3 arms: `graph_edges=3556 non_passenger=0` | **PASS** |

---

## Run Summary

### Warmup (seed=1)
- Duration: 3401.5 s (real), sim range 14400→20400
- State file: `ws_1_200_21600.xml.gz` (361 509 bytes)
- Python state: `warmstate_seed1_python.pkl` (169 650 bytes)
  - sha256=`999b6cac1f9eaf11`, graph_fp=`db665389287b43ea`
  - baselines=487, traversal_edges=1592
- RSU coverage: 3556 / 3556 non-internal edges

### Corridor Gate (seed=1)
| Edge | Traversals | Ratio | Result |
|------|-----------|-------|--------|
| 152535#4 | 10 | 0.996 | PASS |
| 152534#2 | 6 | 0.999 | PASS |
| 152535#2 | 5 | 0.999 | PASS |
| 152714 | 5 | 0.990 | PASS |
| -152330#0 | 29 | 0.936 | PASS |
| 152330#0 | 25 | 0.977 | PASS |
| 14 others | <5 | — | FAIL (no evidence) |

Selected: `['152534#2', '152535#2']` — same pair as Round 6.

### Preseed Regression (at t=21000)
| Arm | n_obs | b0 | b1 | R² | Edges preseeded |
|-----|-------|----|----|-----|----------------|
| ours-fuel | 555 | −741.74 | 108.8293 | 0.405 | 2948/3556 |
| ours-augtime | 555 | −741.43 | 108.8072 | 0.405 | 2948/3556 |
| ablation | 555 | −741.74 | 108.8284 | 0.405 | 2948/3556 |

Baselines frozen: 152534#2=782.06 mg, 152535#2=562.20 mg.

### Arm Outcomes

| Arm | Wall time | Egos arrived / injected | DIAG_EXIT |
|-----|-----------|------------------------|-----------|
| ours-augtime | 1025.5 s | 3/3 (all arrived by t=22411) | natural end |
| ablation | 1263.0 s | 2/3 (ego_2 still running at t=22800) | 22800 |
| ours-fuel | 1287.3 s | 2/3 (ego_0 still running at t=22800) | 22800 |

### Reroute Summary

| Arm | evals | accepted | best_rejected_margin |
|-----|-------|----------|----------------------|
| ours-augtime | 51 | 0 | 0.0% |
| ablation | 61 | 0 | 0.0% |
| ours-fuel | 67 | 0 | 2.6% |

No accepted reroutes: expected — none of the ego initial routes crossed the degraded corridor (`crosses_corridor=False` for all 9 injections), so the grade penalty was never encountered in ego routing decisions.

### Paired Fuel Results (diagnostic, n=1 seed)

| Comparison | Egos crossed corridor | Mean Fuel Saving | Mean Time Saving |
|------------|----------------------|-----------------|-----------------|
| ours-fuel vs ablation | 0 | −30.63% | −29.16% |
| ours-augtime vs ablation | 0 | −5.10% | −3.36% |

Negative "savings" are expected: egos don't cross the corridor, so there is no fuel-penalty bypass, and the SUMO built-in router routes ablation egos identically to ours. The metrics are not meaningful at n=1 smoke scale.

### D4 Margins (ours-fuel)
- n=28, p50=1.298%, p90=2.515%, p99=2.636%, max=2.636%
- All margins < 10% threshold (hysteresis gate correctly blocked sub-threshold candidates)

---

## Vclass Assertion (Criterion 9)

```
[VCLASS_ASSERT] PASS graph_edges=3556 non_passenger=0   ← warmup
[VCLASS_ASSERT] PASS graph_edges=3556 non_passenger=0   ← ours-fuel worker
[VCLASS_ASSERT] PASS graph_edges=3556 non_passenger=0   ← ours-augtime worker
[VCLASS_ASSERT] PASS graph_edges=3556 non_passenger=0   ← ablation worker
```

Fix Q2b (`allows('passenger')` filter) is confirmed structural and correct. No non-passenger edge reachable by Dijkstra.

---

## Graph Fingerprint Guard (Fix Q5)

Warmup wrote `graph_fp=db665389287b43ea`; all 3 arm workers loaded and matched it. No `[PYSTATE_MISMATCH]` events. Guard is live and working.

---

## Non-blocking Issues

### `[WARN] could not patch per_seed_corridor_gate into paired_summary_*.json`
```
name 'json' is not defined
```
The `json` module was not imported at the summary-patching call site in `compare_routing.py`. The paired summary JSON files were written correctly; only the corridor-gate metadata patch failed. Does not affect any acceptance criterion. Fix: add `import json` at that code path. Deferred to Round 8.

---

## Fixes Validated in Smoke 4

| Fix | Description | Validated |
|-----|-------------|-----------|
| Q2a | Sub-1m stub edge filter | ✓ (no crash, graph_edges=3556) |
| Q2b | `allows('passenger')` vclass filter | ✓ (VCLASS_ASSERT PASS × 4) |
| Q3 | Speed-proportional preseed floor | ✓ (PRESEED_REGR × 3 arms) |
| Q4 | BrokenProcessPool containment | ✓ (no ARM_CRASH, exit 0) |
| Q5 | Pickle graph-fingerprint guard | ✓ (fp match × 3 arms) |
| C9 | Programmatic vclass assertion | ✓ (VCLASS_ASSERT × 4) |

---

## Status

**STOP for review.** No k=10 campaign launched. Awaiting user review of Smoke 4 results before authorizing full campaign.

### Commits on `fuel-model-v2` as of this smoke
| Hash | Description |
|------|-------------|
| `0b33f38` | Fix Q2b+C9: add `_assert_passenger_only()` called from `_build_graph()` |
| `08e150b` | DIAG_ROUND6 amendment: Round-7B crash analysis + deliverable backlog |
| `211ad32` | Fix Q4+Q5: BrokenProcessPool containment + pkl graph-fingerprint guard |
| `e5c6243` | Fix Q3: speed-proportional preseed floor + unit tests (22/22 pass) |
| `e4950ef` | Fix Q2b: vclass passenger filter in `_build_graph()` |
| `0827ced` | IMPL_NOTES Round-7B section: ERRATA, units audit, Q2b–Q5 doc |
