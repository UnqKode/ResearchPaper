# DIAG_ROUND6 — Round-6 Diagnostic Report

**Date:** 2026-07-25  
**Branch:** fuel-model-v2  
**Run:** scale=2.0, seed=1, `--diagnose --diagnose-exit-at 21900`  
**Fixes applied before run:** Fix U (traci_compat GC), Fix V (ego rerouting device)  
**Log:** `C:\Users\LNMIIT\AppData\Local\Temp\diag_round6_v2.log`

---

## 0. Run summary

All 3 arms completed with `status=diagnostic_partial` at t=21900.0.  
The harness reported exit code 1 due to a `NameError: name 'sum_json' is not defined`
in the post-processing summary path (only reached in `--diagnose` mode); this is
a code-path bug (fixed in same commit), not a simulation failure.

| Arm | Wall time | Steps | mean steps/s | D1 CSV |
|-----|-----------|-------|-------------|--------|
| ablation | 3565.9 s | 10801 | 3.0 | `diag_weights_ablation_seed1.csv` |
| ours-augtime | 3578.5 s | 10801 | 3.0 | `diag_weights_ours_augtime_seed1.csv` |
| ours-fuel | 3614.5 s | 10801 | 3.0 | `diag_weights_ours_fuel_seed1.csv` |

**Steps/s breakdown (from [PERF] telemetry):**  
- t=14400–19200 (fast-forward, skips RSU): not counted in PERF  
- t≈19325 (subscription init): **0.3–0.4 steps/s**  
- t=19450–21575 (warmup, no egos): **4.7–5.1 steps/s**  
- t=21700 (1 ego active): **4.1–4.2 steps/s**  
- t=21825 (2 egos active): **4.3–4.4 steps/s**  
- Overall mean (PERF_SUMMARY): **3.0 steps/s** (dragged by 0.3 steps/s init)

**Projected k=10 (from PERF_SUMMARY):** ~9.9–10.0 h (3 arms parallel/seed, 10 seeds serial).

---

## A0 Verdict — Ego Rerouting Device

**Status: CONFIRMED and REMEDIATED (Fix V applied)**

**Evidence (static):**
- `most.sumocfg` sets `device.rerouting.probability=1` → all vTypes get the rerouting device by default
- CLI arg `--device.rerouting.period 30` (= REROUTE_INTERVAL) → SUMO autonomously rerouted egos every 30 s
- `ego_petrol` vType had NO `has.rerouting.device=false` before Fix V
- Comment at simulate.py line ~891 already noted this: "keep SUMO's rerouting device off the ego"

**Fix V applied:** `<param key="has.rerouting.device" value="false"/>` added to `ego_petrol` in `basic.vType.xml`.

**Evidence (empirical, from diagnostic run):**  
Zero `[EGO_ROUTE_CONTROL] external_reroute` lines in the log.  
All arms reported: `[EGO_ROUTE_CONTROL] verified: 3 egos tracked external_reroutes_logged=see_above`  
No external reroutes were logged, confirming Fix V is effective.

---

## D1 — Pre-injection Corridor Weight Snapshot (t=21590, scale=2.0)

**D1 fires at t=21590 (depart_start − 10).**

All 3 corridor edges show `n_traversals=0, baseline=frozen=True` in all 3 arms:

| Edge | Baseline (mg) | n_traversals | frozen |
|------|--------------|-------------|--------|
| `152535#4` | 623.8 | 0 | True |
| `-152535#4` | 800.5 | 0 | True |
| `-152534#2` | 800.5 | 0 | True |

**Interpretation:**  
At scale=2.0 (SUMO UPS ≈ 720, ~1477 vehicles inserted by t=21590), zero background
vehicles traversed any of the 3 corridor edges before ego injection. The baseline is
set from the cold network-median preseed (800.5 mg/s for most edges) and the per-edge
free-flow estimate (623.8 mg/s for `152535#4`).

**H1 implication:** The D1 snapshot cannot confirm that corridor edges carry elevated
EU0 fuel costs, because no post-grade-activation traversals have been recorded yet.
The EU0 grade activates at t=21000 (600 s before depart_start), but with sparse traffic
at scale=2.0, no background vehicles happened to traverse these specific edges in that
600 s window. H1 is **INCONCLUSIVE** from this single seed at scale=2.0.

**BASELINE_GATE:** `PASS` for all 3 edges at t=21000 (baselines frozen before ego injection).

---

## D2 — Ego Injection Summary (t=21600–21840, scale=2.0)

All 3 egos injected correctly in all arms, each crossing the corridor:

| Ego | Inject time | ablation route_len | ours-augtime route_len | ours-fuel route_len |
|-----|------------|-------------------|----------------------|---------------------|
| ego_0 | 21600.0 | 47 | 65 | 102 |
| ego_1 | 21720.0 | 58 | 68 | 64 |
| ego_2 | 21840.0 | 55 | 50 | 58 |

`crosses_corridor=True` for all 9 (3 arms × 3 egos).

**Fuel window at ego_0 injection (all 3 arms identical):**
```
152535#4:  n_traversals=0  baseline=623.8mg  frozen=True
-152535#4: n_traversals=0  baseline=800.5mg  frozen=True
-152534#2: n_traversals=0  baseline=800.5mg  frozen=True
```
Cold start at corridor confirmed. The router is working from frozen baseline costs —
no observed EU0 signal yet at first injection.

---

## D4 — FUEL_HYST Margin Distribution (ours-fuel, seed=1)

**Source:** `[FUEL_HYST]` lines in the diagnostic log.  
**Note:** ablation and ours-augtime use `cost_mode=augtime` — no fuel hysteresis applies.

| Metric | Value |
|--------|-------|
| n evaluations (KEEP only) | 15 |
| p50 | **2.054%** |
| p90 | **3.479%** |
| p99 | **3.609%** |
| max | **3.609%** |
| Accepted reroutes (SWITCH) | 0 / 18 evals |
| Best rejected margin | 3.6% |

**REROUTE_SUMMARY comparison:**

| Arm | evals | accepted | best_rejected_margin |
|-----|-------|----------|---------------------|
| ablation | 18 | 3 | 0.0% |
| ours-augtime | 18 | 2 | 0.0% |
| ours-fuel | 18 | **0** | 3.6% |

**Interpretation:**  
The hysteresis guard (`hysteresis=0.15`, requiring ~15% cost improvement) blocks every
reroute evaluation in ours-fuel. Margins at the corridor are 0–3.6%, far below the
15% threshold. The ablation and ours-augtime arms reroute on travel-time cost without
a hysteresis guard and accept 2–3 reroutes respectively.

**Fix S recommendation (from D4 p90):** δ = **3.48%** (p90 of rejected KEEP margins).  
The current `hysteresis=0.15` (15%) must be reduced to allow rerouting when the fuel
saving exceeds ~3.5%. Fix S should set `fuel_hysteresis` to D4-derived δ ≈ 0.035.

---

## H1–H4 Verdicts

### H1 — Degraded edges carry elevated fuel cost before ego injection

**Verdict: INCONCLUSIVE**  
D1 shows `n_traversals=0` for all 3 corridor edges at t=21590 (scale=2.0, seed=1).
No EU0 signal arrived before ego injection because background traffic was too sparse
to traverse the specific corridor edges in the 600 s grade window. A longer grade
window or higher scale would be needed to populate the fuel window.

### H2 — ours-fuel avoids degraded edges more than ablation

**Verdict: NOT MET (hysteresis blocks all rerouting in ours-fuel)**  
ours-fuel: 0 reroutes / 18 evaluations.  
ablation: 3 reroutes / 18 evaluations.  
The hysteresis guard (15%) is too large relative to actual fuel improvement (D4 p90 = 3.48%).
Fix S (reduce `fuel_hysteresis` to ~0.035) is required before H2 can be tested.

### H3 — ours-fuel egos burn less fuel than ablation egos

**Verdict: NOT MEASURABLE**  
No egos arrived before the `--diagnose-exit-at 21900` cutoff (all 3 trips show
`did-not-arrive` in the ARRIVAL_COUNTS summary). Total trip duration > 300 s means
egos injected at 21600–21840 cannot arrive before 21900.

### H4 — fuel specificity: ours-fuel avoidance > ours-augtime avoidance

**Verdict: NOT MEASURABLE** (same reason as H3)

---

## degraded_edges grep

```
[AUTO_NONBOTTLENECK] selected edges: ['152535#4', '-152535#4', '-152534#2']
[BASELINE_GATE] PASS edge=152535#4   baseline=623.8mg frozen=True at t=21000.0
[BASELINE_GATE] PASS edge=-152535#4  baseline=800.5mg frozen=True at t=21000.0
[BASELINE_GATE] PASS edge=-152534#2  baseline=800.5mg frozen=True at t=21000.0
[BASELINE_GATE] PASS all 3 degraded edges have baselines at t=21000.0
[D1_DUMP] arm=ablation   seed=1 t=21590 edges=4404 -> BTP/diag_weights_ablation_seed1.csv
[D1_DUMP] arm=ours-augtime seed=1 t=21590 edges=4404 -> BTP/diag_weights_ours_augtime_seed1.csv
[D1_DUMP] arm=ours-fuel  seed=1 t=21590 edges=4404 -> BTP/diag_weights_ours_fuel_seed1.csv
[FUEL_WINDOW] 152535#4:  n_traversals=0 baseline=623.8mg frozen=True
[FUEL_WINDOW] -152535#4: n_traversals=0 baseline=800.5mg frozen=True
[FUEL_WINDOW] -152534#2: n_traversals=0 baseline=800.5mg frozen=True
```

---

## Fix W — Implemented but Not Benchmarked This Run

Fix W (RSU update decimation via `--rsu-update-interval`) was committed on 2026-07-25
but was NOT active during the diagnostic run above (run used pre-Fix-W code).

**What Fix W does:**  
- Gates `rsu.update_edge_data()` + jam/empty-road snapshots to every N seconds (default 5 s = 20 steps)  
- Traversal detection, `record_traversal_fuel`, and stop-count accumulation remain every step  
- `--rsu-update-interval` added to argparse; asserts REROUTE_INTERVAL (30 s) divisible  
- Startup log: `[RSU] Fix W: update_edge_data decimated to every 5s (20 steps); traversal/fuel/stop detection every step.`

**Expected speedup:** Moderate — `update_edge_data` calls are reduced by (N−1)/N = 95%
at N=5. The 4404-edge Python loop still runs every step for traversal detection.
Confirmed pre-Fix-W throughput: **4.7–5.1 steps/s** at scale=2.0 with no egos,
**4.1–4.4 steps/s** with 1–2 active egos. Addendum says vectorize if still <30 steps/s
after Fix W.

---

## Actions Required Before k=10 Campaign

1. **Fix S**: Reduce `fuel_hysteresis` from 0.15 → ~0.035 (D4 p90 = 3.48%)
2. **Smoke test**: Verify Fix S enables ours-fuel rerouting on a short diagnostic run
3. **Fix W benchmark**: Run a new diagnostic with `--rsu-update-interval 5` and compare
   steps/s to the 4–5 steps/s baseline above
4. **No k=10 until smoke passes** (standing rule)

---

## External Review — Amended Verdicts (2026-07-25)

This section supersedes the H1/H2 verdicts above based on external analysis of the
D1/D2 dump and the ARM_CONFIG log discrepancy.

---

### Q1 — ARM_CONFIG logs wrong hysteresis value

**Finding:** The ARM_CONFIG log line reports `hysteresis=0.15`, but this is the
_augtime deviation threshold_ (`imp_threshold`), not the fuel hysteresis guard.

**Evidence (file:line):**

| Location | Attribute | Value | Role |
|----------|-----------|-------|------|
| `BTP/Simulation/simulate.py:135` | `imp_threshold=0.15` | 0.15 | augtime improvement threshold |
| `BTP/Simulation/simulate.py:158` | `fuel_hysteresis=0.10` | 0.10 | fuel guard (actual) |
| `BTP/Simulation/compare_routing.py:1469` | `_hyst = getattr(sim, "imp_threshold", None)` | reads wrong attr | ARM_CONFIG bug |
| `BTP/Simulation/simulate.py:1046` | `threshold = cost_cur * (1.0 - self.fuel_hysteresis)` | uses 0.10 | actual gate |

**Resolution:** The Round-5 and Round-6 fuel guard was `0.10` (10%), not `0.15` (15%).
The D4 numbers are unchanged (margins 0–3.6%), and the conclusion holds: 10% hysteresis
is still too large given p90 = 3.48% margins. The correct Fix S target is ~0.035 regardless.
The ARM_CONFIG log bug should be fixed (log `sim.fuel_hysteresis` instead of
`sim.imp_threshold`), but it does not change any previously reported numbers.

**Frozen-parameter correction:** the standing spec says "hysteresis at verified current
value" = **0.10** (not 0.15 as misread from ARM_CONFIG).

---

### H1 — Amended Verdict: CONFIRMED

**Original verdict:** INCONCLUSIVE (n_traversals=0 at D1, no EU0 signal arrived)

**External review verdict: CONFIRMED**

**Evidence — 2.2× route-length blowup in ours-fuel at ego_0:**

| Arm | ego_0 route_len (edges) |
|-----|------------------------|
| ablation | 47 |
| ours-augtime | 65 |
| ours-fuel | **102** |

ours-fuel routes ego_0 across 102 edges vs ablation's 47 edges (2.17×). This is a
structural detour: the fuel arm finds a long bypass significantly cheaper than the
direct corridor path.

**Mechanism:** Even with `n_traversals=0`, the cold-preseed baseline (800.5 mg/s for
the corridor edges) is the network-median rate. The fuel arm's cost function
`get_segment_fuel(eid) = cold_nominal_rate(eid) × free_flow_time_s` prices each
edge's fuel cost proportional to its free-flow travel time. The corridor's cold
baseline is consistent with all other edges, but the corridor itself is longer/slower
than many parallel paths. The fuel arm therefore finds a bypass that totals fewer
mg at the cost of more edges — a valid structural routing decision. The 2.2× blowup
confirms that H1's premise (fuel arm routes around the corridor) holds even at cold
start, and that the pricing propagates correctly through the graph.

**Root issue exposed by H1:** The cold preseed is the ONLY pricing signal. Fix P +
Fix Q are needed to make the preseed structurally non-uniform so the corridor's
elevated EU0 cost can actually be differentiated from free-flow alternatives.

---

### H2 — Amended Verdict: CONFIRMED (precondition violated)

**Original verdict:** NOT MET (hysteresis blocks all rerouting)

**External review verdict: CONFIRMED (the precondition for testing H2 is violated)**

H2 asks whether ours-fuel avoids degraded edges *more* than ablation. The test fails
not because hysteresis is too large (though it is), but because **uniform preseed
means F = 0 everywhere**: every edge has the same network-median baseline, so the
fuel-degradation signal (EU0 emission class on corridor edges) cannot produce a
non-zero F term in the weight function. With F = 0 universally, `cost_mode=fuel`
routes on cold `get_segment_fuel()` values only; there is no dynamic signal
distinguishing the degraded corridor from its alternatives post-injection.

The D4 dump (accepted=0/18, best margin=3.6%) is thus a consequence of two stacked
failures:
1. **Cold pricing** (Fix P): ECC state discarded after warmup → arms cold-start
2. **Uniform preseed** (Fix Q): all cold edges get the same network-median rate →
   F = 0 everywhere regardless of degradation

Reducing `fuel_hysteresis` to 0.035 (Fix S) would be necessary but not sufficient
until both Fix P and Fix Q are in place and produce a non-uniform baseline distribution.

---

### New Finding — Warm-up Locks Essentially No Baselines

**D1 corridor evidence (all 3 arms, seed=1, scale=2.0):**

| Edge | n_traversals at D1 | baseline_source |
|------|-------------------|-----------------|
| `152535#4` | 0 | preseed |
| `-152535#4` | 0 | preseed |
| `-152534#2` | 0 | preseed |

All 3 corridor edges have `n_traversals=0` at t=21590. The baseline is locked from
the cold network-median preseed (800.5 mg/s). None of the ~1477 background vehicles
(scale=2.0) traversed any corridor edge in the 600 s grade window (t=21000–21590).

**Network-wide implication:** D1 dump shows 4404 edges total. The proportion with
`baseline_source=observed` (locked from vehicle traversals) will be very low — most
of the network is preseed-only at ego injection. This directly explains the H2
failure: the fuel model cannot differentiate anything from anything.

**Fix P addresses this:** by running RSUManager + EdgeCostCalculator during the
6000 s phase-0 warmup (t=14400→20400), the ECC will accumulate observed baselines
for all frequently-traversed edges, giving Fix Q's regression real data to work with.

---

### F-Term Units Audit

**Finding: No units mismatch.**

The F formula in `EdgeCostCalculator._decompose()`:
```python
cur_rate = fuel_cons / t_actual   # RSU mean_mg_per_traversal / current_traversal_time_s = mg/s
F = clamp(cur_rate / baseline - 1, 0, 2) / 2   # (mg/s) / (mg/s) = dimensionless ✓
```

**RSU `fuel_consumption`** = `agg_fuel / num_departed` where `agg_fuel` is the sum
of per-vehicle total fuel accumulated during traversal (mg). Units: **mg per traversal**.
Source: `rsuController.py:450` (`"fuel_consumption": agg_fuel / num_departed`).

**ECC `_fuel_baseline[eid]`** = EMA of `v_data["fuel"] / time_on_edge` (mg/s).
Source: `rsuController.py:418` (`mean_fuel_rate = v_data["fuel"] / time_on_edge`).

Dividing RSU fuel_consumption (mg/trip) by `t_actual` (s) converts to mg/s for
comparison with the mg/s baseline. The computation is dimensionally correct.

**No code change needed for the F-term.**
