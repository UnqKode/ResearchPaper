# K=10 Definitive Fuel-Mode Results
**Date:** 2026-06-22  
**Branch:** MultiProcessingwithDynamicRouting  
**Run file:** `run_k10_fuel.bat` → `k10_fuel.txt`  
**Analysis artifact:** `paired_summary_20260622_093318_ours_vs_ablation.json`, `paired_per_seed_20260622_093318_ours_vs_ablation.csv`

---

## Experimental Setup

| Parameter | Value |
|-----------|-------|
| Seeds | 1–10 (K=10) |
| OD pairs (n) | 25 per seed |
| Paired trips | 24 per seed (trip 11 excluded — see Non-arrivals) |
| Scale | 2.0 (~2400 vehicles peak) |
| Depart start | 21600 s (6 AM) |
| Road condition | `grade` on edges 153391#0, 153391#1 |
| Teleport threshold | 300 s |
| Reroute cadence | 30 s (both arms) |
| Routing engine | Our Dijkstra (both arms — same engine, confirmed) |
| **ours arm** | `cost_mode=fuel` (edge weight = get_segment_fuel mg) |
| **ablation arm** | `cost_mode=augtime`, α=β=γ=0 (edge weight = L/avg_speed) |
| α / β / γ | 1.0 / 8.0 / 10.0 (ours arm only) |
| Seed parallelism | 3 (3 seeds × 2 arms = 6 concurrent SUMO instances) |

---

## Per-Seed Table

| Seed | n_paired | Fuel % saving | Time % saving |
|------|----------|--------------|--------------|
| 1    | 24       | −19.65%      | −18.56%      |
| 2    | 24       | −15.96%      | −16.62%      |
| 3    | 24       | −13.24%      | −19.66%      |
| 4    | 24       | −17.91%      | −17.16%      |
| 5    | 24       | −15.06%      | −19.75%      |
| 6    | 24       | −20.63%      | −21.05%      |
| 7    | 24       | −11.83%      | −16.10%      |
| 8    | 24       | −19.28%      | −21.83%      |
| 9    | 24       | −12.89%      | −14.81%      |
| 10   | 24       | −13.14%      | −14.16%      |
| **Mean** | | **−15.96%** | **−17.97%** |

Negative = ours (fuel-weighted) uses less fuel / less time than ablation (time-weighted).  
All 10 seeds are negative on both metrics.

---

## Two-Level Statistics (N = 10 seeds, df = 9)

### Fuel (source: `paired_summary_*.json`)
| Statistic | Value |
|-----------|-------|
| Mean saving | −15.96% |
| 95% CI | [−19.92%, −14.13%] |
| Paired t-test p | 3.21 × 10⁻⁷ |
| Wilcoxon p | 0.00195 |
| Cohen's d_z | −4.202 |

### Time (computed from `paired_per_seed_*.csv`)
| Statistic | Value |
|-----------|-------|
| Mean saving | −17.97% |
| 95% CI | [−19.84%, −16.10%] |
| Paired t(9) | −21.77 |
| Wilcoxon p | 0.00195 (all 10 seeds same sign) |
| Cohen's d_z | −6.886 |

Neither CI includes zero. Both metrics significant at p < 0.002.

---

## Gate C — Degraded Corridor Avoidance

Edges: 153391#0, 153391#1 (grade condition)

| Arm | Avoided | Crossed | Avoidance rate |
|-----|---------|---------|----------------|
| ours (fuel) | 78 / 240 | 162 / 240 | **32.5%** |
| ablation (time) | 47 / 250 | 203 / 250 | **18.8%** |

Fuel-weighted routing avoids the degraded corridor **1.73× more often**.

---

## Calibration — DRIVEN_MATCH

Cost function calibrated before K=10 run (see `calib_check.txt`).  
Mean predicted/realized ratio = **0.890** across 10 seeds  
(predicted = sum of get_segment_fuel() on actually-driven route; realized = fuel_abs from tripinfo).  
Within 20% for ~21.7/24 paired egos per seed. Verdict: CALIBRATED.

---

## Non-Arrivals

| Arm | Non-arrivals | Cause |
|-----|-------------|-------|
| ours (fuel) | 1/25 (trip 11) every seed | First edge at capacity at departure → 0 edges driven |
| ablation (time) | 0/25 every seed | Time-weighted Dijkstra picks a different (uncongested) entry edge |

Trip 11 failure is consistent across all 10 seeds and is a spawn-blocking issue on the congested entry edge, not a calibration problem. Excluded from all paired stats.

---

## Interpretation

**Rule 2 — Case 1 applies:**  
|time saving| (17.97%) > |fuel saving| (15.96%)

The fuel router finds routes that are shorter/faster AND use less fuel, but the dominant effect is route quality (congestion avoidance), not fuel-specific intelligence. Both arms use the same Dijkstra engine at the same reroute cadence with the same vehicle placement; the only difference is edge weights. The fuel router steers egos away from the degraded corridor (32.5% vs 18.8% avoidance), which happens to be a congestion bottleneck — so time savings naturally accompany fuel savings.

**What the CI supports:**  
Fuel-weighted routing reliably reduces both fuel consumption and travel time vs pure travel-time routing by ~14–20% (fuel) and ~16–20% (time) across diverse traffic seeds. The fuel savings are real and statistically robust (p = 3.21 × 10⁻⁷, d_z = −4.2). The claim that savings are *fuel-specific* rather than *route-quality-specific* is not supported by this experiment alone.

**Gate C finding:**  
The 1.73× higher corridor avoidance rate is consistent with fuel-weight routing actively penalising the degraded segment, but cannot be disentangled from congestion avoidance effects without a separate controlled experiment.

---

## Files

| File | Contents |
|------|----------|
| `k10_fuel.txt` | Full simulation + analysis stdout (11,269 lines) |
| `paired_summary_20260622_093318_ours_vs_ablation.json` | Fuel stats, Gate C, run metadata |
| `paired_per_seed_20260622_093318_ours_vs_ablation.csv` | Per-seed fuel/time/CO2 values |
| `paired_per_ego_20260622_093318_ours_vs_ablation.csv` | Per-ego paired results |
| `calib_check.txt` | Calibration run (seed 1, DRIVEN_MATCH check) |
