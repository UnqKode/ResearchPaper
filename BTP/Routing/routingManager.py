import sumolib
import networkx as nx

class NetworkBuilder:
    def __init__(self, net_file="MoSTScenario/scenario/in/most.net.xml"):
        self.net_file = net_file
        self.net = sumolib.net.readNet(net_file)
        # MultiDiGraph preserves parallel edges (multiple road segments that share
        # the same from/to intersection pair) — a requirement for real networks like
        # MoST. A DiGraph would silently drop duplicates, corrupting edge_id lookups.
        self.graph = nx.MultiDiGraph()
        self._build_graph()

    def _build_graph(self):
        """
        Builds a NetworkX directed graph from the SUMO network for routing.
        Nodes are intersections, edges are road segments.
        Internal/special edges (IDs starting with ':') are skipped because
        traci.vehicle.setRoute rejects routes containing them.
        """
        for edge in self.net.getEdges():
            # Skip internal edges (within intersections) and special edges.
            # These have IDs starting with ':' and are rejected by setRoute.
            if edge.isSpecial() or edge.getFunction() == "internal":
                continue
            # Fix Q2a: Skip sub-1m stub edges (libsumo C-abort risk).
            if edge.getLength() < 1.0:
                continue
            # Fix Q2b: Skip edges that don't allow passenger vehicles.
            # Pedestrian paths, bike lanes, bus-only roads — ego_petrol
            # (vClass=passenger) cannot enter them; routing through them causes
            # a libsumo C-level abort.  Empirically verified: allows('passenger')
            # is False for 153152#1 (crash edge, 1.4 m/s), True for all arterials.
            if not edge.allows('passenger'):
                continue
            from_node = edge.getFromNode().getID()
            to_node = edge.getToNode().getID()
            edge_id = edge.getID()
            length = edge.getLength()
            speed_limit = edge.getSpeed()

            # Initial setup: Base weight is just the distance (free-flow)
            self.graph.add_edge(from_node, to_node,
                                edge_id=edge_id,
                                length=length,
                                speed_limit=speed_limit,
                                weight=length, # This will be overwritten dynamically
                                fuel_tier=1)
        self._assert_passenger_only()

    def update_graph_weights(self, global_map):
        """
        Pulls dynamic weights from the GlobalMap and updates the NetworkX graph.
        Run this every time you update your GlobalMap weights.
        Iterates with keys=True because self.graph is a MultiDiGraph (parallel
        edges between the same node pair each have a distinct integer key).
        """
        for u, v, k, data in self.graph.edges(keys=True, data=True):
            edge_id = data['edge_id']

            # Fetch the calculated formula weight from map_global.py
            dynamic_weight = global_map.get_weight(edge_id)

            # If the edge hasn't been evaluated yet (returns infinity),
            # fall back to the static physical length in free-flow seconds.
            if dynamic_weight == float('inf'):
                dynamic_weight = data['length'] / data.get('speed_limit', 13.89)

            # Update this specific parallel edge's weight
            self.graph[u][v][k]['weight'] = dynamic_weight

    def _assert_passenger_only(self):
        """Criterion 9: assert zero non-passenger edges leaked into the routing graph.

        Called once at the end of _build_graph() in every worker.  Raises
        AssertionError with the offending edge IDs if any non-passenger edge is
        found; logs [VCLASS_ASSERT] PASS/FAIL to stderr.
        """
        import sys as _sys
        non_pass = [
            data["edge_id"]
            for _, _, data in self.graph.edges(data=True)
            if not self.net.getEdge(data["edge_id"]).allows("passenger")
        ]
        n = self.graph.number_of_edges()
        if non_pass:
            msg = (f"[VCLASS_ASSERT] FAIL: {len(non_pass)} non-passenger edges "
                   f"in routing graph: {non_pass[:5]}"
                   f"{'...' if len(non_pass) > 5 else ''}")
            _sys.stderr.write(msg + "\n")
            _sys.stderr.flush()
            raise AssertionError(msg)
        _sys.stderr.write(f"[VCLASS_ASSERT] PASS graph_edges={n} non_passenger=0\n")
        _sys.stderr.flush()

    def get_dijkstra_route(self, source_node, target_node):
        """
        Runs Dijkstra's algorithm based on current dynamic weights.
        Returns a list of EDGE IDs (TraCI requires edges, not nodes, for routing).
        When multiple parallel edges exist between a node pair, the minimum-weight
        one is chosen, consistent with how Dijkstra selected that pair.
        """
        try:
            # 1. Get the path of nodes using Dijkstra
            node_path = nx.dijkstra_path(self.graph, source=source_node, target=target_node, weight='weight')

            # 2. Convert the node path into an edge path for SUMO.
            # For each consecutive node pair, pick the cheapest parallel edge so
            # the chosen edge_id matches the weight Dijkstra relied on.
            edge_path = []
            for u, v in zip(node_path[:-1], node_path[1:]):
                # choose the cheapest parallel edge u->v
                best_key = min(self.graph[u][v], key=lambda kk: self.graph[u][v][kk]['weight'])
                edge_path.append(self.graph[u][v][best_key]['edge_id'])

            return edge_path

        except (nx.NetworkXNoPath, nx.NodeNotFound, nx.NetworkXError) as e:
            # NetworkXNoPath: nodes exist but are disconnected.
            # NodeNotFound:   src/dst node not in the graph (e.g. the ego is on
            #                 an edge dynamically created after a teleport, whose
            #                 endpoint node was never added). This used to escape
            #                 _reroute_ego's `except TraCIException` and crash the
            #                 whole scenario (HP-3); now it degrades gracefully to
            #                 "keep the current route".
            print(f"Warning: no route {source_node} -> {target_node} ({type(e).__name__})")
            return []

    # --- Utility Methods ---

    def get_graph(self):
        return self.graph

    def get_intersections(self):
        return [node.getID() for node in self.net.getNodes() if node.getType() != "dead_end"]

    def get_all_edges(self):
        return [edge.getID() for edge in self.net.getEdges() if edge.getFunction() != "internal"]