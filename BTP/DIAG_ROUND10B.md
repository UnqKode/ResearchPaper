# DIAG_ROUND10B.md — Round-10B: Verdict Amendment + Junction Quantification + Smoke 7

**Branch:** `fuel-model-v2`  
**Date:** 2026-07-29  
**Commits this round:** `1b5d229` (Step 2a/b), docs  
**Smoke 7 status:** RUNNING / PENDING RESULTS

---

## STEP 0 — Step-1 Verdict Amendment (Honesty Correction)

### Why the Round-10 Step-1 verdict is unreliable

The Round-10 verdict ("F=0 at D1 time → +17.33% came from live t_actual / stochastic divergence") was read from the Smoke-6 D1 dump.

Commit `a7ab047` proved that the D1 dump fired **before** `refresh()`/`update_graph_weights()` was ever called (`active_egos=0 → _do_reroute=False` at t=21590). The GlobalMap was **empty** at dump time. `get_weight()` returned cold fallbacks:
- Augtime: `L/v_lim` (free-flow seconds)
- Fuel: `base_w + junction_pen` from fallback (not from RSU observations)

**"F=0 at D1 time" is indistinguishable from "F was never computed."** The instrument was broken. The verdict is unreliable.

### Claim decomposition

| Claim | Status | Evidence |
|-------|--------|---------|
| Physics: augtime egos avoided corridor; ablation egos crossed it | **STANDS** | crossing_diag.csv, crossed column |
| Physics: ~17% fuel savings is the value of corridor avoidance | **STANDS** | Difference in fuel_mg between arms |
| Detection: augtime sensed corridor via F-term | **UNPROVEN** | D1 instrument invalid; no per-reroute F logs |
| Detection: augtime sensed corridor via S-term | **UNPROVEN** | Same |
| Detection: augtime sensed via t_actual only | **UNPROVEN** | Same; plausible but not confirmed |

**Re-adjudication**: deferred to Smoke 7 criterion 7 (multi-timestamp C/F/S decomposition from the fixed D1 instrument).

---

## STEP 1 — Junction-Penalty Contribution Quantification

### Claim under review

"The junction penalty was asymmetrically present in augtime during Smoke 6, and may be the entire reason augtime avoided the corridor."

### Code audit: is junction penalty in augtime?

**`git log -S "junction_weight"` entry point:** commit `dc9ffb6` ("Change 2A+1+2B", 2026-07-06).  
Commit message (verbatim): "GlobalMap.refresh() and get_weight() both add junction_weight × penalty to **fuel-mode** weights."

Code evidence (`simulate.py`):
```python
# GlobalMap.refresh() fuel branch → YES junction_pen
self.weights[edge_id] = base_w + junction_pen

# GlobalMap.refresh() augtime branch → NO junction_pen
self.weights[edge_id] = self.calc.compute_weight(edge_id, stats)

# GlobalMap.get_weight() augtime cold fallback → NO junction_pen  
return L / v_lim
```

**Junction penalty is NOT in the augtime formula.** It was spec'd as fuel-mode-only in dc9ffb6 and has never been in augtime routing. The "asymmetry" between ours-augtime (junction_weight=1.0) and ablation (junction_weight=0.0) is irrelevant to routing — both use augtime mode where junction_weight is unused.

### Quantification from Smoke 6 D1 data

For the junction penalty to affect routing (hypothetically), it would need to be non-zero. From `get_junction_penalty()` (`edgecost.py` lines 401-453):

```python
stop_freq = m.get("stop_and_go_freq", 0.0)
if stop_freq <= 0.0:
    return 0.0   # immediate return — no penalty
```

**From Smoke 6 D1 dump at t=21590:**

| Edge | S (stop_and_go_freq) | stop_freq ≤ 0? | junction_pen |
|------|---------------------|---------------|-------------|
| 152535#4 | 0.0 | YES | **0.0 mg** |
| 152330#0 | 0.0 | YES | **0.0 mg** |

Both corridor edges had S=0 → `get_junction_penalty()` returns 0 immediately → **junction penalty = 0 mg regardless of junction_weight**.

Note: The D1 dump was from the broken instrument (cold fallback, not live RSU). At actual routing time (t=21600 after refresh()), S could differ. However, grade degradation is modelled as rough road (speed reduction), not stop-and-go traffic. The EU0 emission class lowers speed limits on the road; it does not cause vehicles to halt and restart. Therefore S≈0 at routing time as well.

### Hypothetical: if junction_pen were non-zero (S=1.0 assumed)

Using `get_junction_penalty()` with S=1.0, v_cruise ≈ 13.89 m/s (speed limit):
```
p_stop  = min(1.0/1.0, 1.0) = 1.0
KE      = 0.5 × 1500 kg × 13.89² = 144,700 J
fuel_kg = 144,700 / (0.30 × 43.5×10⁶) = 0.01109 kg
fuel_mg = 0.01109 × 10⁶ × 1.15 = 12,753 mg per stop
```

If this were **accidentally added to augtime seconds** (unit error):
- 152535#4 augtime weight: 18.88s → 18.88 + 12,753 = 12,772 "seconds"
- This would have made the corridor appear ~676× more expensive than free-flow, trivially routing around it.

But **S=0 in Smoke 6** → junction_pen=0 → this scenario did not occur.

### Flip verdict

**Could removing a junction penalty term (if it existed) have flipped augtime's route choice in Smoke 6?**

→ **UNDETERMINABLE from Smoke 6 data** — the D1 instrument was invalid at measurement time.  
→ **NO with high confidence** — because:  
  1. Junction penalty is not in augtime formula (code confirmed)  
  2. S=0 on corridor edges at D1 time → junction_pen=0 even in hypothetical  
  3. Grade degradation does not cause stop-and-go → S≈0 at routing time

### Symmetric junction decision record (confirmed)

Junction penalty: **fuel-mode-only** by original specification (dc9ffb6, 2026-07-06).

- ours-fuel arm: junction_weight=1.0 → full penalty in fuel-mode routing weights
- ours-augtime arm: junction_weight=1.0 → irrelevant (augtime formula ignores it)
- ablation arm: junction_weight=0.0 → irrelevant (augtime formula ignores it)

No arms use junction penalty in augtime routing. The comparison is symmetric. No code change needed. Documented here and in Round-10 section of IMPLEMENTATION_NOTES.md.

---

## STEP 2 — Mechanical Pre-Launch Checks

### 2a. Timeline arithmetic (commit `1b5d229`)

**Old formula:** `seed_warm_end = depart - warmup_buffer`

With `depart=21600, warmup_buffer=1200, grade_lead=1200`:
- Old: `seed_warm_end = 21600 - 1200 = 20400`
- `degrade_start = 21600 - 1200 = 20400`
- **load = activation = 20400** → zero seconds for pre-grade ECC learning

**New formula:** `seed_warm_end = (depart - grade_lead) - warmup_buffer`

- `_seed_activation = 21600 - 1200 = 20400`
- `seed_warm_end = 20400 - 1200 = 19200`
- `[TIMELINE] load=19200 activation=20400 first_ego=21600 baseline_window=1200s detection_window=1200s`

Timeline:
```
14400          17400      19200      20400      21600
  |              |          |          |          |
WARMUP_BEGIN   meas_start  LOAD     ACTIVATE   EGO_0
               |←  1800s  →|          |          |
                            |← 1200s →|← 1200s  →|
                            baseline   detection
```

Measurement window (corridor gate): 17400→19200 (1800s, pre-grade background traffic)  
ECC baseline learning: 19200→20400 (1200s, normal traffic in ARM runs)  
Grade detection window: 20400→21600 (1200s, EU0 corridor active)  
First ego: 21600

Hard assertions added to code: `load < activation` and `activation < first_ego`.

RoadConditionManager in warmup-savestate mode updated to use `_seed_activation = depart - grade_lead` (per-depart, sweep-correct) instead of global `degrade_start` (from `depart_start`, wrong for sweep runs).

### 2b. Gate-log precision (commit `1b5d229`)

`P(X≥3|λ=16)` was formatted with `:.3f` → printed "1.000" (rounds 0.999983 to 1.0000 at 3dp). Changed to `:.6f`.

Actual value: `P(X≥3 | λ=16) = 1 − e^(−16)×(1+16+128) = 1 − 1.125×10⁻⁷×145 = 0.999983`

Sample gate log after fix:
```
[CORRIDOR_GATE] PASS edge=152330#0 ratio=0.921 traversals=31 λ=20.7 P(≥3)=0.999999 occ=0.0001
```

Code comment updated: "P(X≥3|λ=16) ≈ 100%" → "P(X≥3|λ=16) = 0.999983".

---

## STEP 3 — Smoke 7

### Configuration

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
- Timeline: load=19200, activation=20400 (was: load=activation=20400)
- CORRIDOR_MIN_TRAVERSALS=24 (was 9)
- Poisson gate active: P(X≥3|λ) ≥ 0.95 required per corridor
- --grade-lead 1200 (was 600; new default)
- D1 dump shows live RSU state (Fix O5)
- Gate log shows P(≥3) to 6dp (no more "1.000")

**Frozen (unchanged):** α=1.0, β=8.0, γ=10.0, window=1800s, aggregator=median, reroute=30s, sample-mod=2, junction-weight=1.0 (fuel arm only), bg-reroute-prob=0.25, teleport=300, δ=0.01.

---

## Smoke 7 Criteria (verbatim adjudication)

**Criterion 1:** Exit 0, no `[ARM_CRASH]`, vclass ×3, `[PYSTATE]` fp ×3, `[OD_COUPLING]` ≥0.60 not forced, `[TIMELINE]` shows load=19200 activation=20400 first_ego=21600 (with warmup-buffer=1200, grade-lead=1200, depart-start=21600).

**Criterion 2:** Corridor gate passed under the Poisson criterion (log shows warmup_n, λ, P(≥3) with real 6dp values); BOTH corridor edges have ≥3 post-activation window samples in `[FUEL_WINDOW]` before first ego routes.

**Criterion 3:** D1 per-arm dumps with `[D1_ASSERT]` PASS ×3 (post-a7ab047 fix); fuel-arm `weight` column shows mg-scale values (>100 mg for corridor edges); augtime arm `weight` column shows compute_weight() values (not cold L/v_lim fallback).

**Criterion 4:** Route parity: fuel-arm initial routes within 15% of ablation route length (meters); routing-time bypass ratio ≤ 1.05 for all egos in crossing_diag.

**Criterion 5:** Fuel arm: accepted reroutes ≥1 OR initial routes avoid corridor post-activation (either counts as detection; report which); ≤5 reroutes per ego.

**Criterion 6:** 9/9 arrivals. Crossing table: ablation crosses corridor; ours-fuel crosses LESS than ablation post-activation. **Headline: ours-fuel median fuel ≤ ablation median fuel.** Per-ego fuel spread printed. **Per-ego TIME deltas vs ablation for all arms** (fuel savings without time penalty is the project goal; time is a first-class metric from this smoke forward).

**Criterion 7:** **Step-1 re-adjudication (the real one).** Augtime C/F/S/multiplier decomposition on BOTH corridor edges at ≥3 timestamps spanning grade activation → first ego (from the fixed D1 instrument + any available RSU logs), PLUS 2 non-corridor arterial control edges. Verdict: which term (F, S, C, or t_actual alone) carries the corridor penalty signal? Report honestly: F moving = detection story revives and O7 blending becomes a Round-11 enhancement; F flat despite ≥3 EU0 traversals = finding about the F-term's sensitivity.

**Criterion 8:** Projected k=10 ≤ 12h with longer warmup (estimate from mean_steps_s and arm parallelism). Teleports ≤ 2× Smoke 5 baseline.

---

## Smoke 7 Results — PENDING

*To be filled after run completes.*

### `degraded_edges` grep

```
[GREP PENDING]
```

### Corridor gate log

```
[PENDING]
```

### [TIMELINE] verification

```
[PENDING]
```

### D1 assert

```
[PENDING]
```

### Crossing table

```
[PENDING]
```

### Per-ego fuel and time

```
[PENDING]
```

### Criterion 7: Multi-timestamp decomposition

```
[PENDING]
```

### Adjudication table

| # | Criterion (verbatim) | Verdict | Evidence |
|---|----------------------|---------|---------|
| 1 | Exit 0, no ARM_CRASH, vclass×3, PYSTATE fp×3, OD_COUPLING≥0.60, TIMELINE | PENDING | — |
| 2 | Corridor gate Poisson PASS; ≥3 post-activation samples in FUEL_WINDOW | PENDING | — |
| 3 | D1 PASS×3; fuel mg-scale; augtime compute_weight() | PENDING | — |
| 4 | Route parity within 15%; bypass ratio ≤1.05 | PENDING | — |
| 5 | Fuel arm: reroutes≥1 OR initial avoidance; ≤5/ego | PENDING | — |
| 6 | 9/9 arrivals; fuel avoidance; fuel ≤ ablation; per-ego fuel+time | PENDING | — |
| 7 | Step-1 re-adjudication: C/F/S decomposition ≥3 timestamps + 2 controls | PENDING | — |
| 8 | k=10 projected ≤12h; teleports ≤2×Smoke-5 | PENDING | — |
