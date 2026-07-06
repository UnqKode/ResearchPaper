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

### Why run `select_nonbottleneck_grade_edges` before OD selection?

The OD-coverage filter inside the selector needs the candidate edge set. Running it before
OD generation means the OD pairs can then be built with `--targeted-od` to ensure ego trips
cross the selected corridor. The alternative (pick edges after OD) would break the OD
coverage guarantee.

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
```

Expected output: `14/14 tests passed`

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
- `fuel_specificity` (OLS intercept + CI = fuel-specific effect)
