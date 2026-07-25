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
