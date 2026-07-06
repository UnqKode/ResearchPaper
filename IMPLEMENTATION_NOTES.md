# Implementation Notes — Fuel-Aware Routing Model (Phase 3)
**Branch:** `fuel-model-v2`  **Date:** 2026-07-06

## Overview

Three upgrades to the fuel-aware routing model, implemented in order 2A → 1 → 2B → 3.
Two commits: one for 2A+1+2B (core model), one for Change 3 (experiment design).

---

## Design Decisions

### Why `(sim_time, mg)` tuples in `_traversal_fuel`?

The original deque stored bare floats. Adding a timestamp field enables write-time eviction
of samples from an earlier traffic regime without touching the `maxlen` window semantics.
`clear_traversal_fuel` and `reset()` remain unchanged because they call `dq.clear()` which
works regardless of element type.

### Why median as the default aggregator?

A single delayed/recalculated-route vehicle can produce an outlier traversal observation
(e.g., 5000 mg for a normally-150-mg edge). Median suppresses this without losing recency
ordering. `wmean` preserves the original behavior for regression studies.

### Why clamp `v_cruise = min(v_lim, v_obs)` in the junction penalty?

`avg_speed` from SUMO's rolling window can slightly exceed the posted speed limit due to
acceleration. Clamping prevents the KE estimate from reflecting behaviour above the design
speed, while still using the observed speed when it reflects real slow-downs.

### Why branch `_evaluate_and_reroute` at `cost_mode == "fuel"` rather than inside the acceptance gate?

The augtime improvement gate (`improvement > imp_threshold`) compares route costs in CFS
units. Fuel-mode costs are in mg and the acceptance semantics are different (hysteresis
fraction vs. improvement fraction). Separating the two code paths avoids an abstraction
that would obscure both policies.

### Why is `route_predicted_fuel` on `Simulation` rather than `GlobalMap`?

`GlobalMap.get_weight` already adds the junction penalty when computing the graph weight.
`route_predicted_fuel` is a diagnostic helper used only at reroute decision time. Putting
it on `Simulation` keeps it next to `_reroute_fuel_mode` without changing the `GlobalMap`
interface.

### Why `auto-nonbottleneck` instead of always auto-selecting?

The original `auto` selector deliberately picks high-occupancy bottleneck edges — that is
the correct choice for congestion-avoidance experiments. Change 3 needs a different causal
structure: high fuel burn, low time impact. The two selectors are complementary, not
replacements.

### Why run `select_nonbottleneck_grade_edges` AFTER plain OD generation? (Fix 3)

Review fix corrected the original ordering. The selector now runs after
`generate_od_pairs` in `main()` so it receives the real campaign OD list for
coverage filtering (preference 1 from the Fix 3 spec). The `_nonbottleneck_deferred`
flag in `main()` records this two-step flow:

1. `generate_od_pairs` → `_plain_od` (no degraded edges needed yet)
2. `select_nonbottleneck_grade_edges(od_list=_plain_od)` → `degraded_edges`
3. If `--targeted-od`: re-run `select_targeted_od_pairs` with the real corridor

This breaks the circular dependency (targeted-OD needs edges; selector needs ODs)
by using the plain OD list for coverage estimation and targeted-OD for the campaign
itself. The selector also generates a 50-pair fallback internally if `od_list=None`
is passed directly.

---

## Integration Points

| Component | Changed |
|-----------|---------|
| `BTP/RSU/edgecost.py` | `record_traversal_fuel` signature, `get_segment_fuel` aggregator, `get_junction_penalty` method, new constructor params |
| `BTP/RSU/rsuController.py` | Thread `sim_time` into `record_traversal_fuel` |
| `BTP/Simulation/simulate.py` | `GlobalMap.refresh` + `get_weight` fuel paths, `route_predicted_fuel`, `_reroute_fuel_mode`, `_evaluate_and_reroute` branching |
| `BTP/Simulation/compare_routing.py` | `select_nonbottleneck_grade_edges`, `_arm_params`, CLI flags, three-arm campaign loop, `analyze_paired_results` gates + decomposition |
| `BTP/run_k10_fuelspec.bat` | New: three-arm experiment launcher |
| `BTP/test_segment_fuel.py` | New: 14 unit tests (no SUMO/TraCI) |
| `BTP/test_nonbottleneck_select.py` | New (round 2): 14 tests for Fixes 1-7 |

---

## CLI Flags Added

| Flag | Default | Effect |
|------|---------|--------|
| `--fuel-aggregator` | `median` | `wmean` / `median` / `trimmed` |
| `--junction-weight` | `1.0` | Multiplier on junction penalty; `0.0` disables |
| `--fuel-hysteresis` | `0.10` | Min fractional saving to accept reroute in fuel mode |
| `--arms` | `""` | Comma-separated arm names; blank = legacy two-arm behavior |
| `--degraded-edges auto-nonbottleneck` | — | Probe-sim non-bottleneck selector |

---

## Backward Compatibility

- `--junction-weight 0` restores pre-Change-1 fuel weights exactly
- `--fuel-aggregator wmean` reproduces pre-Change-2A aggregation exactly
- `--arms` absent → `arms_to_run` derived from `--baseline` as before
- Existing augtime runs are unaffected (fuel-mode branch not entered)
- Checkpointing and multi-processing unchanged

---

## Running the Tests

```powershell
cd C:\Users\LNMIIT\Desktop\SumoSimulation\ResearchPaper\BTP
conda run --no-capture-output -n ml python test_segment_fuel.py
conda run --no-capture-output -n ml python test_nonbottleneck_select.py
```

Expected output: `14/14 tests passed` for each file (28 total).

---

## Running the Three-Arm Campaign

```powershell
cd C:\Users\LNMIIT\Desktop\SumoSimulation\ResearchPaper\BTP
.\run_k10_fuelspec.bat
```

Logs: `k10_fuelspec_run.txt` / `k10_fuelspec_run.err.txt`

Output files per arm pair (example for ours-fuel vs ablation):
- `paired_per_ego_<stamp>_ours-fuel_vs_ablation.csv`
- `paired_per_seed_<stamp>_ours-fuel_vs_ablation.csv`
- `paired_summary_<stamp>_ours-fuel_vs_ablation.json`

The summary JSON includes:
- `fuel_sav_mean`, `dur_sav_mean`, `co2_sav_mean`
- `fuel_stats` (t-test, Wilcoxon, Cohen's d_z, 95% CI)
- `gate_c` (avoidance rates + artifact flag)
- `gate_a_prime` (fuel/time ratio on degraded-edge trips)
- `gate_c_prime` (ours-fuel avoidance > base, only for ours-fuel arm)
- `fuel_specificity` (per-seed intercepts, mean, CI, Wilcoxon, equal-time subset,
  seed-level OLS — see Fix 6)

---

## Review Fixes (round 2)

Applied after code review of Change 3. Changes 1, 2A, 2B were approved unchanged.

| Fix | Severity | Location | Summary |
|-----|----------|----------|---------|
| 1 | Critical | `select_nonbottleneck_grade_edges` | Inverted speed filter: `<=speed_ratio_max` → `>=speed_ratio_min=0.85`. Old filter selected congested edges, new selects free-flowing ones. |
| 2 | High | same | Occupancy normalisation: divide by 100 at read time (TraCI returns %). Replace fixed `occ_max` with adaptive p40 threshold computed from probe run. |
| 3 | High | same + `main()` | Replace broken `__import__`/`Simulation.__new__` OD coverage hack with pure `sumolib.net.getShortestPath` loop. `od_coverage_min` 0.05→0.20. Selector call deferred in `main()` to after `generate_od_pairs` so real OD list is passed. |
| 4 | High | same | Two missing candidate gates: length >= 100 m; expected traversals >= 30 (proxy via vehicle*dt / t_freeflow). |
| 5 | Medium | same | Replace hardcoded `probe_port=9188` with `_free_port()`. Unique TraCI label. Probe loop wrapped in try/finally so close always runs. |
| 6 | Medium | `analyze_paired_results` | Replace single 10-point seed-level OLS with two-level analysis: per-seed OLS intercepts, seed-level t-CI + Wilcoxon, equal-time subset (|Δtime%|≤5%) report. Old OLS kept as `seedlevel_ols` secondary readout. |
| 7 | Minor | `simulate.py` | Off-route ego in fuel mode now calls `_reroute_fuel_mode(force=True)` instead of returning early. `force=True` bypasses hysteresis gate, always applies Dijkstra route, logs `FORCE_OFFROUTE`. |

### Fix 3 ordering decision

The selector is now called **after** `generate_od_pairs` in `main()` (preference 1 from
the spec, not the fallback). Implementation: the `_nonbottleneck_deferred = True` flag
delays the selector call to after OD generation. The plain OD list is passed to the
selector for coverage filtering; if `--targeted-od` is also set, OD pairs are then
regenerated with `select_targeted_od_pairs` using the real corridor.
