# DIAG_ROUND10.md — Round-10 Pre-Smoke Analysis

**Branch:** `fuel-model-v2`  
**Date:** 2026-07-29  
**Commits this round:** `a7ab047` (Fix O5), `87ead20` (Fix O6)  
**Status:** STOP — awaiting user authorization before Smoke 7

---

## Context

Smoke 6 result: ours-augtime +17.33% fuel saving; ours-fuel −3.92%.  
Root cause: 152535#4 had 0 post-activation traversals → fuel Dijkstra blind to EU0 degradation.  
Three open items identified: O5 (D1_ASSERT bug), O6 (post-activation data starvation), O7 (fuel arm sensing).

Round-10 task: decompose the +17.33% (Step 1), fix D1_ASSERT (Step 2), blending if F moved (Step 3), Poisson gate (Step 4), Smoke 7 (Step 5).

---

## STEP 1 — Decompose the +17.33%: F or S?

**Question:** Did the augtime arm's +17.33% fuel saving (vs ablation) come from the F-term (live fuel intensity, `VAR_FUELCONSUMPTION`) or the S-term (stop frequency)?

### Evidence: D1 dump at t=21590 (seed=1, ours-augtime)

| edge | C | F | S | multiplier | t_actual | weight (GlobalMap) |
|------|---|---|---|-----------|----------|--------------------|
| 152535#4 | 0 | 0 | 0 | 1.0 | 18.8835 | 18.5688 |
| 152330#0 | 0 | 0 | 0 | 1.0 | 56.2441 | 56.2441 |

Both corridor edges: **F=0, S=0, C=0** at D1 time. The augtime formula `t_actual × (1+α×C+β×F+γ×S)` = `t_actual × 1.0` = `t_actual`. No F or S contribution.

**Why F=0:** 152535#4 had 10 warmup traversals in 1800s → expected post-activation traversals in 600s = 10×600/1800 = 3.33. Actual = 0 (confirmed by `[FREEZE]` log: `n_window=0 at grade activation`). With zero traversals, `fuel_consumption` in RSU subscriptions was never populated → F=0.

**Why weight ≠ t_actual for 152535#4:** weight=18.5688 vs t_actual=18.8835. At D1 time, GlobalMap had never been refreshed (active_egos=0 → `_do_reroute=False` always). `get_weight()` returned the augtime cold fallback: `L/v_lim = 18.5688s` (free-flow time). This is the O5 bug, now fixed in `a7ab047`.

### Step 1 verdict

F and S were **ZERO** on corridor edges at D1 time (t=21590). The D1 dump captures the last GlobalMap state before ego injection; with no refresh() prior to D1, it reflected cold fallback values (L/v_lim), not live RSU values.

**The +17.33% mechanism is live t_actual routing**, not F or S:
- Both ablation and ours-augtime use augtime formula. With F=S=C=0, both route on `t_actual × 1.0`.
- However, at the time of periodic reroutes during the trip, `t_actual` on corridor edges may have been elevated above free-flow due to grade congestion, causing ours-augtime to reroute away.
- The D1 dump at t=21590 showed t_actual slightly elevated on 152535#4 (18.88 vs free-flow 18.57 = +1.7%). This small congestion signal, compounded over multiple reroute cycles and edges, is the likely mechanism.
- Additionally, stochastic divergence between arm SUMO processes (background vehicles take different routes after warmup state) contributes noise.

**Per spec: "build ONLY if Step 1 says F." F did not move. Step 3 (O7 blending) is SKIPPED.**

Note: even if F had moved during the trip (which cannot be confirmed without per-reroute F logs), the D1 dump at t=21590 shows the signal was absent at initial route time. The fuel arm's sensing problem is data starvation (0 traversals → no RSU fuel signal), which O6 (Poisson gate) addresses directly.

---

## STEP 2 — Junction-Penalty Provenance + D1_ASSERT Fix

### 2a. Junction-penalty provenance

**`git log -S "junction_weight"` entry point:** commit `dc9ffb6` ("Change 2A+1+2B: configurable aggregator, junction penalty, fuel hysteresis"), authored 2026-07-06.

**Commit message (verbatim):** "GlobalMap.refresh() and get_weight() both add junction_weight * penalty to fuel-mode weights. --junction-weight 0 restores pre-change behaviour exactly."

**Code evidence (simulate.py):**
```
GlobalMap.refresh() fuel branch (lines 95-97):
    junction_pen = (self.calc.junction_weight
                    * self.calc.get_junction_penalty(edge_id, m))
    self.weights[edge_id] = base_w + junction_pen

GlobalMap.refresh() augtime branch (lines 98-105):
    # NO junction penalty
    self.weights[edge_id] = self.calc.compute_weight(edge_id, stats)

GlobalMap.get_weight() fuel fallback (lines 121-123):
    junction_pen = (self.calc.junction_weight
                    * self.calc.get_junction_penalty(edge_id, m))
    return base_w + junction_pen

GlobalMap.get_weight() augtime fallback (line 124-126):
    return L / v_lim  # NO junction penalty
```

**Conclusion:** Junction penalty is **fuel-mode-only by original spec** (dc9ffb6). It was never intended for augtime routing. The asymmetry between `ours-augtime` (junction_weight=1.0) and `ablation` (junction_weight=0.0) is NOT a routing asymmetry — both arms use augtime mode which ignores junction_weight in routing weight computation. The junction_weight=0.0 on ablation only affects `route_predicted_fuel()` (hysteresis), which is unused by ablation (no fuel-mode rerouting).

**Resolution: DOCUMENT SYMMETRIC DECISION — no code change required.**

Junction penalty is fuel-mode-only by design. The comparison between ours-augtime and ablation is symmetric at the routing level (neither uses junction penalty in routing costs). The junction_weight parameter on ablation is a harmless zero that silences junction-penalty contributions to fuel estimation — consistent with ablation representing SUMO's standard time-optimal routing with no fuel awareness.

### 2b. D1_ASSERT fix (commit `a7ab047`)

**Root cause:** D1 dump fires at t≥21590 via `if diagnose and not _diag_d1_done and sim_time >= (depart_start - 10.0)`. At this point, `active_egos` is empty (egos not injected until t=21600). The `_do_reroute` condition (line 1796) requires `bool(active_egos)` → False → `refresh()` and `update_graph_weights()` were NEVER called before D1.

**Consequence for fuel arm:** `GlobalMap.weights = {}` (empty) → `get_weight(eid)` returns fuel fallback (mg) → D1 writes mg in `weight` column. Routing graph retains `_build_graph()` initial values (L/v_lim, seconds). D1_ASSERT: mg vs seconds → FAIL(5) for all sampled edges.

**Consequence for augtime arm:** `GlobalMap.weights = {}` → `get_weight(eid)` returns L/v_lim (cold fallback) → D1 writes L/v_lim in `weight` column, same as routing graph → D1_ASSERT trivially PASS. BUT: `weight` column showed L/v_lim (free-flow), not live RSU augtime formula — masking the true state.

**Fix applied:** Added to top of `_dump_d1_weights()`:
```python
self.global_map.refresh(self.edges)
self.net_builder.update_graph_weights(self.global_map)
```

This ensures D1 captures live RSU state and routing graph == GlobalMap for all arms. The sync is safe: `_inject_ego_trip()` calls another refresh+update at ego injection time (t=21600), overwriting D1-time values with the t=21600 state.

**Expected outcome after fix:**
- Fuel arm: `weight` column shows mg (from fresh `get_segment_fuel() + junction_pen`). Routing graph shows same mg. D1_ASSERT PASS.
- Augtime arm: `weight` column shows `compute_weight()` result (t_actual × multiplier). If F>0 on any edge, multiplier > 1.0. D1_ASSERT PASS.
- D1 dump at t=21590 now reflects actual RSU state rather than cold fallback — meaningful for diagnosis.

---

## STEP 3 — SKIPPED

F did not move at D1 time. Per spec: "build ONLY if Step 1 says F." O7 live-rate blending is not implemented this round.

---

## STEP 4 — Fix O6: Poisson-Honest Corridor Gate (commit `87ead20`)

### Root cause of data starvation (Smoke 6)

152535#4 had 10 sampled warmup traversals in 1800s measurement window.
- With grade_lead=600s: λ_post = 10 × 600/1800 = **3.33**
- P(X≥3 | λ=3.33) = 1 − e^(−3.33)(1+3.33+5.54) = **64.5%**
- Actual post-activation traversals on 152535#4: **0** (unlucky draw, P=3.6%)
- Fuel arm froze at preseed baseline (641 mg) → no degradation signal → −3.92%

### Fix applied

**CORRIDOR_MIN_TRAVERSALS: 9 → 24**

At 24 warmup traversals with grade_lead=1200s:
- λ_post = 24 × 1200/1800 = **16.0**
- P(X≥3 | λ=16) ≈ **100%** (≫ 95% threshold)

**--grade-lead default: 600 → 1200s**

Doubles the post-activation observation window (20400→21600 instead of 21000→21600).

**Poisson gate added to `_corridor_gate()`:**
```python
lam = warmup_traversals × grade_lead_s / CORRIDOR_MEASURE_WINDOW_S
p_ge3 = 1 − exp(−λ)(1 + λ + λ²/2)
poisson_ok = p_ge3 >= 0.95
```

Gate now requires ALL of: `warmup_traversals ≥ 24` AND `P(X≥3|λ) ≥ 0.95` AND `speed_ratio ≥ 0.85` AND `avg_occ ≤ 0.40`.

`[CORRIDOR_GATE]` log lines now report λ and P(≥3) for every candidate.

### Buffer arithmetic with grade_lead=1200, warmup_buffer=1200

- `warmup_end = depart_start − warmup_buffer = 21600 − 1200 = 20400`
- `degrade_start = depart_start − grade_lead = 21600 − 1200 = 20400`
- **Grade activates immediately at warmup end.** The entire 1200s between warmup completion and ego departure is grade-active observation time.
- Warmup measurement window (18600–20400) is purely pre-grade: gate measures clean baseline traversal rate.

### Retroactive gate on Smoke 6 corridor

Applying new gate to Smoke 6 152535#4 (10 warmup traversals, grade_lead=1200):
- trav_ok: 10 < 24 → **FAIL**
- λ = 10 × 1200/1800 = 6.67, P(X≥3) = 96.2% → poisson_ok: 96.2% ≥ 95% → PASS
- But trav_ok FAIL → **REJECTED**

With the new gate, 152535#4 would have been rejected and a different corridor edge selected. Smoke 7 will reveal which edges pass.

---

## STEP 5 — Smoke 7 PENDING

**STOP — awaiting user authorization.**

Standing rules: "STOP for review after each smoke" and "No k=10 campaign without explicit user authorization."

When authorized, Smoke 7 command:
```
python -u -m Simulation.compare_routing --paired --n 3 --seeds 1 --scale 2.0 \
  --arms ours-fuel,ours-augtime,ablation \
  --road-condition grade --degraded-edges auto-nonbottleneck --n-degraded 2 \
  --targeted-od \
  --warmup-savestate --diagnose --diagnose-exit-at 24000 \
  --bg-reroute-prob 0.25 --depart-start 21600 \
  --warmup-buffer 1200 --grade-lead 1200 \
  --vehicle-sample-mod 2 --rsu-update-interval 5 \
  --teleport 300
```

**Changes from Smoke 6:**
- `--grade-lead 1200` (was 600; new default)
- `CORRIDOR_MIN_TRAVERSALS=24` (was 9; in code)
- Poisson gate active (new; in code)
- D1 dump now shows live RSU state (Fix O5; in code)

**Frozen (unchanged):** α=1.0, β=8.0, γ=10.0, window=1800s, aggregator=median, reroute=30s, sample-mod=2, junction-weight=1.0, bg-reroute-prob=0.25, teleport=300, δ=0.01.

---

## Smoke 7 Criteria (verbatim, for adjudication after run)

**Criterion 1:** `[CORRIDOR_GATE]` log shows `gate=passed` (not fallback) for seed=1. Selected corridor edges listed.

**Criterion 2:** Both selected corridor edges have `warmup_traversals ≥ 24` AND `P(≥3) ≥ 0.95` in `[CORRIDOR_GATE]` log.

**Criterion 3:** `[FUEL_WINDOW]` log at first ego departure shows both corridor edges with `n_traversals ≥ 3`.

**Criterion 4:** D1 CSV present for all 3 arms; fuel-arm `weight` column shows mg-scale values (> 100 mg for corridor edges); `[D1_ASSERT]` shows PASS for all 3 arms.

**Criterion 5:** ours-fuel accepted reroutes ≥ 1 for at least one ego.

**Criterion 6:** ours-fuel median fuel saving vs ablation > 0% (positive saving).

**Criterion 7:** ours-augtime median fuel saving vs ablation ≥ 5%.

**Criterion 8:** No ego teleported during active routing (crossing_diag `drift_ratio` < 5.0 for all rows).

---

## Open Items Carried Forward

**O7 (Fuel arm sensing via F-channel):** DEFERRED. F=S=C=0 at D1 time; mechanism for augtime advantage is live t_actual, not F/S. Blending would require F to move during the trip. Need per-reroute F-log to confirm before building. STOP per spec.

**O8 (Step 1 confirmation):** Add per-reroute GlobalMap F/S dump (small log line) to confirm during Smoke 7 whether F ever becomes nonzero on corridor edges during the trip. This would confirm or refute O7.

---

## Commits This Round

| Hash | Description |
|------|-------------|
| `a7ab047` | Fix O5: sync GlobalMap+graph before D1 dump/assert |
| `87ead20` | Fix O6: Poisson-honest corridor gate + grade-lead 1200s |
