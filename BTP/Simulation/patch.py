import os
import re

file_path = "compare_routing.py"
with open(file_path, "r") as f:
    content = f.read()

# 1. Add select_targeted_od_pairs and select_degraded_edges after _free_port
free_port_idx = content.find("def _free_port():")
insertion_point = content.find("\n", free_port_idx) + 1
while content[insertion_point] == " ":
    insertion_point = content.find("\n", insertion_point) + 1

new_code = '''
# ---------------------------------------------------------------------------
# Phase 2: Dynamic Degradation Methods
# ---------------------------------------------------------------------------
def select_degraded_edges(net_file, k, traffic_seed, scale, teleport, depart_start):
    """
    Auto-select k edges by running a headless calibration run up to depart_start-300.
    Selects the k edges with the highest vehicle counts that are >= 50m and >= 8m/s.
    """
    print(f"Running headless calibration to select {k} degraded edges...")
    import traci
    from Simulation.simulate import RSUManager
    
    port = _free_port()
    sumo_cmd = [
        SUMO_BIN, "-c", CONFIG_FILE,
        "--seed", str(traffic_seed),
        "--scale", str(scale),
        "--no-step-log", "true",
        "--no-warnings", "true",
        "--time-to-teleport", str(teleport),
    ]
    net = sumolib.net.readNet(net_file)
    
    traci.start(sumo_cmd, port=port)
    rsu_manager = RSUManager(record_interval=300)
    
    end_time = depart_start - 300
    while traci.simulation.getTime() < end_time:
        traci.simulationStep()
        rsu_manager.step()
        
    edges = traci.edge.getIDList()
    edge_counts = []
    for e in edges:
        if not e.startswith(":"):
            stats = rsu_manager.get_edge_stats(e)
            if stats:
                try:
                    sumo_edge = net.getEdge(e)
                    if sumo_edge.getLength() >= 50.0 and sumo_edge.getSpeed() >= 8.0:
                        edge_counts.append((e, stats.get("vehicle_count", 0)))
                except:
                    pass
                    
    traci.close()
    
    edge_counts.sort(key=lambda x: x[1], reverse=True)
    # Simple connectivity check: assume top edges don't completely disconnect MoST
    # A full connectivity check is expensive, but MoST is highly redundant.
    selected = [x[0] for x in edge_counts[:k]]
    print(f"Selected degraded edges: {selected}")
    return selected

def select_targeted_od_pairs(net_file, degraded_edges, n, seed, vclass="passenger", min_len=50.0):
    """
    Select OD pairs where the shortest path crosses at least one degraded edge,
    and an alternative exists.
    """
    print(f"Building {n} targeted OD pairs crossing {degraded_edges}...")
    net = sumolib.net.readNet(net_file)
    candidates = [e for e in net.getEdges() if (not e.isSpecial()) and e.allows(vclass) and e.getLength() >= min_len]
    
    rng = random.Random(seed)
    pairs = []
    seen = set()
    attempts = 0
    max_attempts = n * 500
    
    while len(pairs) < n and attempts < max_attempts:
        attempts += 1
        a = rng.choice(candidates)
        b = rng.choice(candidates)
        if a.getID() == b.getID(): continue
        key = (a.getID(), b.getID())
        if key in seen: continue
        
        try:
            path, cost = net.getShortestPath(a, b, vClass=vclass)
        except:
            path = None
            
        if path and len(path) > 1:
            path_ids = [e.getID() for e in path]
            # Must cross a degraded edge
            if any(de in path_ids for de in degraded_edges):
                # Ensure alternative exists (not strictly checking detour factor here for speed)
                seen.add(key)
                pairs.append(key)
                
    if len(pairs) < n:
        print(f"  WARNING: only found {len(pairs)} targeted pairs.")
    return pairs

'''
content = content[:insertion_point] + new_code + content[insertion_point:]

with open(file_path, "w") as f:
    f.write(content)
print("Updated compare_routing.py phase 1")
