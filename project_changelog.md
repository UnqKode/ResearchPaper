# MoST Fuel-Routing Simulation Project Changelog

## Context
This project aims to demonstrate that a custom fuel-aware routing algorithm (incorporating real-time RSU metrics for Congestion (C), Fuel (F), and Stops (S)) can outperform standard travel-time-based routing (like SUMO's default) under specific dynamic traffic conditions in the Monaco SUMO Traffic (MoST) scenario.

## Phase 1: Structural Fixes & Gate Check
**Goal:** Fix the harness to ensure fair, identical evaluations between the `ours` (fuel-aware) and `ablation` (travel-time) arms, and determine if the penalty functions organically under standard congestion.

**Key Fixes:**
1. **Dijkstra Fallback Structural Bug:** Fixed the heuristic/fallback logic in `NetworkBuilder.update_graph_weights` to ensure the structural graph costs perfectly matched the baseline when multipliers were 1.0. This removed an artificial 15% length penalty that was being applied to the ablation arm.
2. **Snapshot Hysteresis Fix:** Fixed the hysteresis locking mechanism. The system previously overwrote the baseline value in the route snapshot during reroutes, preventing it from detecting changes.
3. **Multi-Processing Safety:** Enforced isolated TraCI ports and fixed `WORKER_TIMEOUT_S` limits to prevent premature termination during long N=100 sweeps.

**Gate Check Results (N=100, Scale=2.5x):**
- **Hypothesis:** Under standard heavy background congestion, the fuel penalty is either mathematically dead OR travel-time routing inherently coincides with the fuel-optimal route.
- **Empirical Findings:** The penalty mathematically fires (`max_route_multiplier = 1.1595`), but the mean multiplier across 2.3 million edge queries was `1.00006`. `baseline_locked` fallbacks accounted for ~23% of queries.
- **Conclusion:** The baseline shortest-time path naturally dodges the most congested areas, thereby naturally maximizing fuel efficiency in this topology. The penalty is real but entirely swamped by the structural routing weights. Phase 1 proves that standard congestion does not trigger the C/F/S penalty aggressively enough to force divergence.

## Phase 2: Degraded-Road Experiment
**Goal:** Introduce mid-simulation environmental degradations (potholes/water-logging or accidents) on targeted edges to force stop-and-go conditions, spiking the HBEFA fuel burn without significantly inflating travel time. This creates a divergence between the travel-time and fuel-aware paths.

**Implementation Rules:**
1. **Physics First:** The degraded edges MUST genuinely burn more fuel in SUMO (Gate A verification).
2. **Symmetry:** Both arms experience the exact same physical degradation.
3. **Honesty Constraint:** The routing algorithm does NOT know which edges are degraded. The penalty MUST propagate organically through the RSU measurements (Fuel -> F term, Stops -> S term).
4. **Behavioural Divergence:** We must prove that `ours` actively avoided the degraded edges while `ablation` drove through them (Gate C verification).

## Phase 3: Fuel-Aware Routing Model Upgrades
**Branch:** `fuel-model-v2` (off `MultiProcessingwithDynamicRouting`)
**Goal:** Strengthen the fuel-routing model with three targeted upgrades to improve physical realism, robustness to observation noise, and causal isolation of the fuel benefit.

### Change 2A — Configurable Fuel Aggregator + Max-Age Eviction
**Files:** `BTP/RSU/edgecost.py`, `BTP/RSU/rsuController.py`

`_traversal_fuel` deque now stores `(sim_time, mg)` tuples. At write time, samples older than `fuel_sample_max_age_s=600.0` s are evicted (left-side pop) so stale observations from a different traffic regime cannot contaminate current estimates.

Three aggregators selectable via `--fuel-aggregator`:
- `wmean` — recency-weighted mean (original behaviour; higher weight to recent samples)
- `median` — (default) robust to outlier traversals
- `trimmed` — drops lowest and highest sample when n≥4

`sim_time` is now threaded from `rsuController.step()` → `record_traversal_fuel()`.

### Change 1 — Junction / Transition Fuel Penalty
**Files:** `BTP/RSU/edgecost.py`, `BTP/Simulation/simulate.py`

`EdgeCostCalculator.get_junction_penalty(edge_id, m)` computes the expected fuel cost of a stop-start at the upstream junction:

```
KE = 0.5 × m_veh × v_cruise²
fuel_mg = KE / (eta_engine × LHV_gasoline) × 1e6 × idle_overhead_factor
penalty = p_stop × fuel_mg
```

where `p_stop = clamp(stop_and_go_freq, 0, 1)` and `v_cruise = min(speed_limit, avg_speed)`.

`GlobalMap.refresh()` (fuel mode) and `GlobalMap.get_weight()` (fuel fallback) both add `junction_weight × junction_penalty` to the base fuel weight. `--junction-weight 0` restores pre-change behaviour exactly.

Constants: `m_veh=1500 kg`, `eta_engine=0.30`, `LHV_gasoline=43.5 MJ/kg`, `idle_overhead_factor=1.15`.

### Change 2B — Fuel-Mode Route Hysteresis
**Files:** `BTP/Simulation/simulate.py`, `BTP/Simulation/compare_routing.py`

In fuel mode, `_evaluate_and_reroute()` branches into `_reroute_fuel_mode()` after the deviation and cooldown checks. A reroute is accepted only if:

```
route_predicted_fuel(candidate) < route_predicted_fuel(current) × (1 − fuel_hysteresis)
```

`route_predicted_fuel(edge_list)` sums `get_segment_fuel + junction_penalty` per edge. Default `fuel_hysteresis=0.10` (10% savings required). Logs `[FUEL_HYST]` REROUTE / KEEP messages. `--fuel-hysteresis` CLI flag.

Augtime mode is unchanged — the existing deviation + improvement-threshold logic runs as before.

### Change 3 — Non-Bottleneck Grade Corridor + Three-Arm Analysis
**Files:** `BTP/Simulation/compare_routing.py`

**`select_nonbottleneck_grade_edges()`:** Runs a 900-step headless probe SUMO, collects per-edge occupancy and speed, then applies a 4-filter pipeline:
1. Occupancy in [occ_min=0.05, occ_max=0.60] — non-bottleneck
2. avg_speed / speed_limit ≤ speed_ratio_max=0.75 — speed-depressed
3. OD coverage ≥ od_coverage_min=0.05 — on ego routes
4. Not a critical arterial (occ > 0.60 excluded)

`--degraded-edges auto-nonbottleneck` invokes this selector in `main()`.

**Three-arm campaign:** `--arms ours-fuel,ours-augtime,ablation` runs all three arms against the same OD list, seeds, and physics. Ablation is the reference; each `ours-*` arm is analyzed separately via `analyze_paired_results(..., ours_label=arm_name)` producing distinct output files.

**`analyze_paired_results()` additions:**
- **Gate A′:** On trips that cross a degraded edge: fuel_ratio≥2×, time_ratio<1.10
- **Gate C′:** ours-fuel avoidance rate > base arm avoidance rate (labeled in summary JSON)
- **Equal-time OLS decomposition:** `Δfuel% = a + b·Δtime%`; intercept `a` = fuel-specific effect with 95% CI. Written to `summary["fuel_specificity"]`.

**`run_k10_fuelspec.bat`:** Seeds 1-10, n=25, scale=2.0, teleport=300, grade mode, auto-nonbottleneck k=2, three arms, median/0.10/1.0 flags.

**`test_segment_fuel.py`:** 14 unit tests covering all three changes. 14/14 pass without SUMO/TraCI.

### Non-negotiable invariants maintained
- Routing algorithm never reads `RoadConditionManager.degraded_edges`
- All arms experience identical physics, OD lists, seeds, reroute cadence
- All edge weights remain positive and finite
- Augtime mode, checkpointing, and multiprocessing unchanged
