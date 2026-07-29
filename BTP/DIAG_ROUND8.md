# DIAG_ROUND8.md — Round-8 Hypothesis Test + D1 Instrument Fix

**Date:** 2026-07-29
**Branch:** `fuel-model-v2`
**HEAD at write:** see commit hash below
**Smoke:** none this round (hypothesis refuted after Step 1; Step 5 / Smoke 6 not triggered)

---

## STEP 1 — Corridor-Crossing Analysis (Smoke 5 artifacts)

### Source data

- **D2 route CSV** (`diag_routes_ours_fuel_seed1.csv`, `…_augtime_…`, `…_ablation_…`): initial routes at injection, appended across all runs
- **Crossing diag** (`crossing_diag_20260726_140732_1.csv`): Smoke 5 driven routes at ego arrival
- **Smoke 5 log** (`bx2i00r31.output`): `[EGO_ROUTE]`, `[REROUTE_SUMMARY]` lines

Smoke 5 rows are the last 3 rows per arm file (route_len values 62/21/53 for ours-fuel, 68/17/53 for ours-augtime, 60/17/43 for ablation — matching Smoke 5 `[EGO_ROUTE]` logs).

### Crossing Table — Smoke 5 (seed=1)

| Ego | Arm | Initial route contains corridor edge? | Driven route crosses corridor? | Per-ego fuel (mg) |
|-----|-----|---------------------------------------|-------------------------------|------------------|
| ego_0 | ours-fuel | **No** (route_len=62, crosses_corridor=False) | **No** (crossing_diag crossed=False) | 617,953 |
| ego_1 | ours-fuel | **No** (route_len=21) | **No** | 193,984 |
| ego_2 | ours-fuel | **No** (route_len=53) | **No** | 587,622 |
| ego_0 | ours-augtime | **No** (route_len=68) | **No** | 450,139 |
| ego_1 | ours-augtime | **No** (route_len=17) | **No** | 148,484 |
| ego_2 | ours-augtime | **No** (route_len=53) | **No** | 588,093 |
| ego_0 | ablation | **No** (route_len=60) | **No** | 408,219 |
| ego_1 | ablation | **No** (route_len=17) | **No** | 148,498 |
| ego_2 | ablation | **No** (route_len=43) | **No** | 786,652 |

Evidence for "driven route": `crossing_diag_20260726_140732_1.csv` — all 9 rows show `crossed=False`. Our Dijkstra: `[REROUTE_SUMMARY]` accepted=0 in all 3 arms.

### Bypass-cost ratios (from crossing_diag, Smoke 5)

`ratio = chosen_cost / bypass_cost` — how far each ego is above the cheapest non-corridor route:

| Ego | ours-fuel ratio | ours-augtime ratio | ablation ratio |
|-----|-----------------|--------------------|----------------|
| ego_0 | 1.112 (+11.2%) | 1.072 (+7.2%) | 1.024 (+2.4%) |
| ego_1 | 1.371 (+37.1%) | 1.049 (+4.9%) | 1.059 (+5.9%) |
| ego_2 | 1.007 (+0.7%) | 1.011 (+1.1%) | 1.013 (+1.3%) |

ours-fuel ego_1 is routing 37% above the optimal no-corridor route. This is not because ego_1 crossed the corridor (crossed=False); it is because the fuel-mode Dijkstra, operating on cold preseed weights (R²=0.405), chose a suboptimal route.

### Hypothesis Verdict: **REFUTED**

The working hypothesis was: *"The +4.18% ours-fuel fuel overhead is attributable to egos being routed through the half-priced degraded corridor (152535#2 unpriced at n=0 post-activation)."*

Evidence against:
1. crossed=False × 9 for all egos, all arms, both initial and driven routes
2. Zero accepted reroutes from our Dijkstra in any arm
3. Bypass ratios: ours-fuel ego_1 is 37% above optimal, not because of corridor traversal but because cold preseed weights mislead the fuel router

The +4.18% is attributable to the fuel-mode Dijkstra making suboptimal initial route choices based on noisy preseed weights (R²=0.405). Without real corridor observations (n_traversals=0 on 152535#2 post-activation), the fuel router has no calibrated data to differentiate routes. The corridors ARE the source of real fuel signals; without egos crossing them, the router is flying blind.

**Root cause (structural):** None of the 3 OD pairs at seed=42 has either corridor edge (`152534#2` or `152535#2`) on any natural path in any arm. This is the same as Open Item O4 from Round-7C. The mechanism (fuel savings from avoiding a priced corridor) cannot engage until the OD pairs are configured to actually route through the corridor.

### Historical note

D2 route row 1 (early run, not Smoke 4/5): ours-fuel ego_0 initial route contained `152535#2` and `crosses_corridor=True`. That run used a different corridor selection (`152535#4` as primary degraded edge). With the CURRENT corridor selection (`['152534#2', '152535#2']`) and CURRENT OD pairs (seed=42), no corridor crossing occurs. The mechanism is achievable but not engaged.

---

## STEP 2 — D1 Instrument Fix (applied regardless of hypothesis outcome)

### Bug (pre-fix)

`_dump_d1_weights()` called `self.calc._decompose(eid, m)` for ALL arms and used `d["weight"]` (augtime formula: `t_actual × (1+α·C+β·F+γ·S)`) as the `weight` column. Result: both `ours-fuel` and `ours-augtime` D1 CSVs showed identical weights regardless of cost mode. Criterion 4 was NOT EVALUATED in every smoke because the dump never reflected fuel-mode weights.

### Fix (commit: see below)

`_dump_d1_weights()` now uses `self.global_map.get_weight(eid)` as the `weight` column — the same call that `update_graph_weights()` uses to populate the routing graph. This is the weight the arm's Dijkstra actually consumes:

- **fuel mode**: warm edge → `get_segment_fuel()` (traversal-window median); cold edge → `cold_nominal_rate × (L/v)` + junction penalty (mg units)
- **augtime mode**: `t_actual × (1+α·C+β·F+γ·S)` + junction penalty (seconds units)

New columns added:

| Column | Content |
|--------|---------|
| `cost_mode` | "fuel" or "augtime" — makes CSV self-describing |
| `augtime_weight` | Always the augtime formula (reference; unchanged from prior `weight`) |
| `fuel_warm` | True if traversal-fuel deque non-empty (warm branch active in fuel mode) |
| `weight` | **THE routing weight this arm's Dijkstra consumes** (was: augtime formula) |

`augtime_weight` is kept so prior analyses (which read `weight` as augtime) can be adapted without data loss.

### Post-write assertion (D1_ASSERT)

After writing the CSV, 5 random edges are spot-checked: `GlobalMap.get_weight(eid)` is compared against the weight stored in the routing graph (`net_builder.graph` edge data). Relative tolerance 1e-4. Output: `[D1_ASSERT] PASS/FAIL eid=… dumped=… graph=…`. Catches any second copy of the formula that drifted from the last `update_graph_weights()` call.

### Unit test

`test_d1_dump_uses_arm_cost_function` (test_segment_fuel.py) — uses stub SimpleNamespace objects (no SUMO dependency):
- fuel arm: verifies `weight` == GlobalMap stub weight (12345, 67890) ≠ `_decompose` result
- augtime arm: verifies `weight` == augtime GlobalMap stub weight (14.4, 10.0)
- Verifies `cost_mode` column, `fuel_warm` True/False, `augtime_weight` present

**24/24 tests pass.**

---

## Round-8 Status

**STOP after Steps 1–2.** Hypothesis refuted; Steps 3–5 (corridor rate gate, Fix S, Smoke 6) are premature.

### What needs to happen in Round 9 (before Steps 3–5 are meaningful)

| ID | Item | Action |
|----|------|--------|
| O4 | OD pairs don't route through corridor at seed=42 | Investigate corridor geography vs OD pair sources/sinks. Either (a) re-generate OD pairs with seed that produces corridor-crossing paths, or (b) move the corridor gate to select edges that ARE on natural paths between OD pairs |
| O1 | 152535#2 gets 0 post-activation traversals in 600s window | Resolve O4 first; once egos cross the corridor, fuel observations accumulate |
| O3 | PERF 6.7–6.9 steps/s << 30 | Report says 4,404 RSU subs vs 3,556 routable edges — Step 5 perf check deferred |

Once O4 is resolved: re-run Steps 3–5 (corridor rate gate, Fix S δ=0.01, Smoke 6). The D1 fix (Step 2) is already applied and will correctly show fuel-arm weights in Smoke 6.

---

## Commits (Round-8)

| Hash | Description |
|------|-------------|
| TBD | Step 2: Fix D1 dump — arm-specific `weight` via GlobalMap, D1_ASSERT, unit test 24/24 |

Prior HEAD: `a599e19` (DIAG_ROUND7C.md)
