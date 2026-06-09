# Code Loopholes and Breaking Points - MoST BTP Project

## Critical Issues (Will Crash/Fail)

### 1. **Missing SUMO_HOME Environment Variable** ❌
**Location:** [main.py](BTP/main.py#L13-L15), [compare_routing.py](BTP/Simulation/compare_routing.py#L54-L56)
**Issue:** No try-catch fallback if SUMO_HOME is not set
```python
if 'SUMO_HOME' in os.environ:
    # ...
else:
    sys.exit("Error: Please declare the environment variable 'SUMO_HOME'")
```
**Breaking Point:** Will exit immediately without installing SUMO or providing alternatives
**Fix:** Document required setup or auto-detect SUMO installation path

---

### 2. **File Path Validation Missing** ❌
**Location:** [main.py](BTP/main.py#L35-L36), [compare_routing.py](BTP/Simulation/compare_routing.py#L97-L98)
**Issue:** No validation that config_file and net_file exist before passing to SUMO
```python
config_file = os.path.join(_root, "scenario", "most.sumocfg")
net_file    = os.path.join(_root, "scenario", "in", "most.net.xml")
# No os.path.exists() check before use
```
**Breaking Point:** If files are missing, SUMO fails with cryptic error messages
**Impact:** Users won't know which file is missing
**Fix:** Add validation:
```python
if not os.path.exists(config_file):
    sys.exit(f"Config file not found: {config_file}")
if not os.path.exists(net_file):
    sys.exit(f"Network file not found: {net_file}")
```

---

### 3. **TraCI Port Conflicts** ❌
**Location:** [main.py](BTP/main.py#L49), [compare_routing.py](BTP/Simulation/compare_routing.py#L137)
**Issue:** Multiple SUMO instances could use the same TraCI port
```python
traci.start(sumo_cmd)  # Default port 8813 not explicitly specified
```
**Breaking Point:** If a previous SUMO instance crashed, next run will fail with "port already in use"
**Impact:** Requires manual kill of stray SUMO processes
**Fix:** Use explicit port or implement port-finding logic:
```python
import socket
def find_free_port():
    with socket.socket() as s:
        s.bind(('', 0))
        return s.getsockname()[1]

port = find_free_port()
sumo_cmd = ["sumo", "-c", config_file, "--remote-port", str(port)]
traci.start(sumo_cmd, port=port)
```

---

### 4. **No Recovery from Incomplete Edge Data** ❌
**Location:** [rsuController.py](BTP/RSU/rsuController.py#L195-L210)
**Issue:** If a vehicle arrives but has no subscription data (edge case), avg_speed could be 0
```python
v_data = self.active_vehicles[edge_id][veh_id]
agg_speed  += veh_avg_speed  # Could be 0
agg_speed / num_departed     # Results in 0 avg_speed
```
**Breaking Point:** In [edgecost.py](BTP/RSU/edgecost.py#L110), division by zero or NaN weights:
```python
v = max(m["avg_speed"], self.v_min)  # v_min=0.1, but clamping happens AFTER check
t_actual = L / v  # Could produce infinite weight if v=0
```
**Fix:** Validate all metrics have sensible values before using them

---

### 5. **Unhandled TraCIException on Vehicle Removal** ❌
**Location:** [simulate.py](BTP/Simulation/simulate.py#L370-L376)
**Issue:** Multiple places call TraCI methods without exception handling
```python
if self.ego_id not in traci.vehicle.getIDList():
    return  # But getIDList() itself can fail
try:
    cur_edge = traci.vehicle.getRoadID(self.ego_id)  # Can throw
except:
    pass  # No catch block shown
```
**Breaking Point:** Unexpected TraCI errors propagate and crash the simulation
**Impact:** One bad TraCI call kills entire run
**Fix:** Wrap all TraCI calls in try-except

---

### 6. **Infinite Loop in Route Finding** ❌
**Location:** [routingManager.py](BTP/Routing/routingManager.py#L59-L62)
**Issue:** No timeout on Dijkstra algorithm
```python
try:
    node_path = nx.dijkstra_path(self.graph, source=source_node, target=target_node, weight='weight')
except nx.NetworkXNoPath:
    print(f"Warning: No valid path found...")
    return []
```
**Breaking Point:** On large graphs (MoST has ~2000 edges), Dijkstra could take seconds to minutes
**Impact:** Simulation step gets blocked, RTF (real-time factor) drops dramatically
**Risk:** If called every 30 steps × 36000 steps, could add hours to runtime
**Fix:** Add timeout or use A* instead of Dijkstra

---

### 7. **Dead-End Edge Assignment Bug** ⚠️
**Location:** [rsuController.py](BTP/RSU/rsuController.py#L133-L159)
**Issue:** Dead-end edges assigned to from_node RSU, but that RSU may not have been created yet
```python
for eid, to_node in edge_to_node.items():
    if eid in self.edge_to_rsu:
        continue  # skip if already assigned
    from_node = edge_from_node.get(eid)
    if from_node in self.rsus:  # ASSUMES from_node RSU exists
        rsu = self.rsus[from_node]
```
**Breaking Point:** If from_node is NOT an intersection, it won't have an RSU → edges get NO routing weight
**Impact:** Dead-end roads always use static weight, never dynamic optimization
**Symptom:** Routes never prefer/avoid peripheral streets even when congested
**Fix:** Ensure all non-dead-end nodes create RSUs

---

## High-Priority Loopholes

### 8. **Fuel Baseline Never Converges** ⚠️
**Location:** [edgecost.py](BTP/RSU/edgecost.py#L58-L66)
**Issue:** `_fuel_baseline` is updated with slow alpha (0.2 down, 0.05 up) but:
```python
def record_vehicle_fuel(self, edge_id, fuel_rate):
    cur = self._fuel_baseline.get(edge_id)
    if cur is None:
        self._fuel_baseline[edge_id] = fuel_rate  # First vehicle sets baseline
        return
    a = self._a_down if fuel_rate < cur else self._a_up  # Asymmetric decay
```
**Problem:** 
- First vehicle ever on an edge completely determines baseline
- If that vehicle is slow, baseline is set HIGH
- Then fast vehicles can never lower F index significantly (α_up = 0.05 too slow)

**Breaking Point:** Fuel index F stays artificially high forever → routes avoid previously-visited edges
**Impact:** Algorithm converges to suboptimal routes due to locked baseline
**Fix:** Initialize baseline with multiple samples, or reset periodically

---

### 9. **Missing Subscription Data (One-Step Lag)** ⚠️
**Location:** [rsuController.py](BTP/RSU/rsuController.py#L192-L203)
**Issue:** Vehicle subscription results lag by one step
```python
if veh_id not in self._subscribed_vehicles:
    traci.vehicle.subscribe(veh_id, _VEH_VARS)
    self._subscribed_vehicles.add(veh_id)

v_sub = veh_results.get(veh_id)
if not v_sub:
    # No data yet (first step after subscribe); skip accumulation.
    continue
```
**Impact:** 
- Every vehicle's first step on an edge is skipped
- For short edges (~50m @ 13m/s = 4 steps), this is 25% data loss
- Short edge stats are severely undersampled

**Fix:** Pre-subscribe all vehicles on entry, or use instantaneous queries

---

### 10. **No Timeout for Per-Trip Simulation** ⚠️
**Location:** [simulate.py](BTP/Simulation/simulate.py#L145-L160)
**Issue:** `per_trip_timeout` is steps, not seconds, but no absolute time limit
```python
while True:
    traci.simulationStep()
    # ...
    if step >= per_trip_timeout:  # 3000 steps = 750 seconds real-time
        self._safe_remove_ego()
        break
```
**Problem:** If simulation is very slow, 3000 steps could take 10+ minutes
**Breaking Point:** User leaves machine unattended, wastes time on stuck vehicle
**Fix:** Add wall-clock timeout:
```python
import time
start_time = time.time()
while time.time() - start_time < 600:  # 10-minute hard limit
```

---

### 11. **Window Size Too Small for Congestion Detection** ⚠️
**Location:** [rsu.py](BTP/RSU/rsu.py#L6), [rsuController.py](BTP/RSU/rsuController.py#L100)
**Issue:** Default window_size=60 steps = 15 seconds
```python
self.edge_data = {
    edge: {
        "avg_speed": deque(maxlen=60),  # Only 15 seconds of history
```
**Problem:** 
- Peak-hour traffic has cycles of 5-10 minutes
- 15-second window misses macro patterns
- Router makes short-term decisions on noise, not trends

**Impact:** Routes oscillate, never find equilibrium
**Fix:** Increase to 600-1200 steps (2.5-5 minutes)

---

### 12. **Division by Zero in Edge Cost** ⚠️
**Location:** [edgecost.py](BTP/RSU/edgecost.py#L109-B112)
**Issue:** No check for avg_speed == 0 before division
```python
v = max(m["avg_speed"], self.v_min)
t_actual = L / v  # v_min = 0.1, but what if m["avg_speed"] is negative?
```
**Problem:** If TraCI returns negative speed (shouldn't but could), t_actual becomes negative
**Impact:** Negative weights break Dijkstra
**Fix:** 
```python
v = max(abs(m["avg_speed"]), self.v_min)  # Use abs()
```

---

## Medium-Priority Issues

### 13. **Missing Bounds Check on Multiplier** ⚠️
**Location:** [edgecost.py](BTP/RSU/edgecost.py#L155-L157)
**Issue:** Multiplier capped at 10.0, but terms could still overflow
```python
multiplier = 1.0 + self.alpha * C + self.beta * F + self.gamma * S
multiplier = min(multiplier, self.max_multiplier)  # max_multiplier = 10.0
```
**If:** alpha=1.0, beta=0.8, gamma=1.5 and C,F,S all approach 1.0
```
multiplier = 1.0 + 1.0 + 0.8 + 1.5 = 4.3  # OK
```
But if params are tuned differently, could exceed 10.0 cap → gets clipped
**Impact:** Non-linear behavior, hard to tune alpha/beta/gamma
**Fix:** Normalize individual terms before summing

---

### 14. **No Logging of Skipped Reroutes** ⚠️
**Location:** [simulate.py](BTP/Simulation/simulate.py#L156-L160)
**Issue:** If reroute fails silently, user doesn't know
```python
def _reroute_ego(self):
    if self.ego_id not in traci.vehicle.getIDList():
        return  # Silent return
    # ... more silent returns
```
**Impact:** Debugging is hard; can't tell if ego is actually being rerouted
**Fix:** Add debug logging:
```python
import logging
logger = logging.getLogger(__name__)
logger.debug(f"Reroute attempt for {self.ego_id}: skipped (not in network)")
```

---

### 15. **No Warm-Up Validation** ⚠️
**Location:** [simulate.py](BTP/Simulation/simulate.py#L168-L177)
**Issue:** Warmup could complete early but edge costs never populate
```python
for _ in range(warmup_steps):
    traci.simulationStep()
    self.rsu_manager.step()
    if traci.simulation.getMinExpectedNumber() <= 0:
        print("[warm-up] network drained early; stopping warm-up.")
        break  # But fuel baseline may not be seeded!
```
**Breaking Point:** If network empties during warmup, fuel baseline is never seeded → F index broken
**Fix:** Log fuel_baseline stats after warmup:
```python
if not self._edge_cost_calc._fuel_baseline:
    print("[WARNING] No fuel baseline seeded after warmup!")
```

---

### 16. **Disconnected Network Not Detected** ❌
**Location:** [compare_routing.py](BTP/Simulation/compare_routing.py#L82-L94)
**Issue:** OD pair generation silently skips unreachable pairs
```python
try:
    path, cost = net.getShortestPath(a, b, vClass=vclass)
except Exception:
    path, cost = None, None
if path and len(path) >= 1 and cost is not None and cost < float("inf"):
    pairs.append(key)
```
**Problem:** If network is fragmented (e.g., ferry routes), valid OD pairs might be ~10 but requested 100
```
WARNING: only found 32 routable pairs (requested 100) after 20000 attempts.
```
**Breaking Point:** A/B comparison is now run on wildly different OD sets
**Fix:** Raise error if < 80% of requested pairs found:
```python
if len(pairs) < 0.8 * n:
    sys.exit(f"Network is fragmented; only {len(pairs)}/{n} reachable pairs!")
```

---

## Low-Priority (Refinement Issues)

### 17. **No Vehicle Type Validation**
**Location:** [main.py](BTP/main.py#L47), [simulate.py](BTP/Simulation/simulate.py#L321)
**Issue:** ego_type must exist in the vType list, but not validated
**Fix:** 
```python
available_vtypes = traci.vehicletype.getIDList()
if self.ego_type not in available_vtypes:
    print(f"WARNING: vType '{self.ego_type}' not found; available: {available_vtypes}")
```

---

### 18. **Missing Step Count Logging**
**Location:** [main.py](BTP/main.py#L56-B59)
**Issue:** No progress indication during long runs
```python
steps = sim.run()
print(f"\nSimulation finished after {steps} steps.")
```
**Fix:** Log progress every N steps in sim.run()

---

### 19. **No Memory Cleanup for Long Campaigns**
**Location:** [rsuController.py](BTP/RSU/rsuController.py#L205)
**Issue:** Vehicle data is deleted when vehicle leaves edge, but active_vehicles dict grows unbounded
```python
self.active_vehicles = {edge_id: {} for edge_id in edges}  # Initialized large
```
**Fix:** Periodic cleanup or use LRU cache

---

### 20. **No Validation of Custom Alpha/Beta/Gamma**
**Location:** [main.py](BTP/main.py#L48)
**Issue:** User can pass invalid coefficients
```python
sim = Simulation(
    ...
    alpha=1.0, beta=0.8, gamma=1.5,  # No range checking
)
```
**Fix:** Validate in EdgeCostCalculator.__init__():
```python
if not (0 <= alpha <= 5):
    raise ValueError(f"alpha must be in [0, 5], got {alpha}")
```

---

## Summary Table

| # | Severity | Issue | Location | Impact |
|---|----------|-------|----------|--------|
| 1 | 🔴 CRITICAL | Missing SUMO_HOME | main.py | Immediate crash |
| 2 | 🔴 CRITICAL | No file validation | main.py, compare_routing.py | Cryptic error |
| 3 | 🔴 CRITICAL | TraCI port conflicts | main.py | Port already in use |
| 4 | 🔴 CRITICAL | Incomplete edge data | rsuController.py | Invalid weights |
| 5 | 🔴 CRITICAL | Unhandled TraCIException | simulate.py | Random crashes |
| 6 | 🔴 CRITICAL | Infinite loop in routing | routingManager.py | Simulation hangs |
| 7 | 🟠 HIGH | Dead-end edge assignment | rsuController.py | Routes never optimize |
| 8 | 🟠 HIGH | Fuel baseline locked | edgecost.py | Suboptimal routes |
| 9 | 🟠 HIGH | Missing subscription lag | rsuController.py | Data loss |
| 10 | 🟠 HIGH | No trip timeout | simulate.py | Hangs forever |
| 11 | 🟠 HIGH | Window size too small | rsu.py | Noisy decisions |
| 12 | 🟡 MEDIUM | Division by zero | edgecost.py | Invalid weights |
| 13 | 🟡 MEDIUM | Multiplier overflow | edgecost.py | Clipping artifacts |
| 14 | 🟡 MEDIUM | No reroute logging | simulate.py | Hard to debug |
| 15 | 🟡 MEDIUM | Warmup not validated | simulate.py | Silent failures |
| 16 | 🔴 CRITICAL | Disconnected network | compare_routing.py | Invalid comparison |
| 17 | 🔵 LOW | No vType validation | main.py | Silent failures |
| 18 | 🔵 LOW | No progress logging | main.py | Looks hung |
| 19 | 🔵 LOW | Memory leak | rsuController.py | Long runs OOM |
| 20 | 🔵 LOW | No param validation | main.py | Unexpected behavior |

---

## Recommendations

**Immediate actions:**
1. Add file existence checks before SUMO launch
2. Wrap all TraCI calls in try-except
3. Add explicit port management for TraCI
4. Add timeout on Dijkstra calls
5. Validate warmup success

**Before next run:**
6. Increase window_size from 60 to 600
7. Add fuel baseline validation
8. Add progress logging

**Longer-term:**
9. Unit tests for weight calculations
10. Integration tests with sample OD pairs
11. Memory profiling for long campaigns
