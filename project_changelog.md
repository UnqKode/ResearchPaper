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
