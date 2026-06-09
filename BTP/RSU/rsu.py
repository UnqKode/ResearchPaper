from collections import deque
import numpy as np

class RSU:
    def __init__(self, intersection_id, connected_edges, window_size=60):
        self.intersection_id = intersection_id
        self.connected_edges = connected_edges
        self.window_size = window_size
        
        # Dictionary to store rolling data for each connected edge
        self.edge_data = {
            edge: {
                "vehicle_count": deque(maxlen=window_size),
                "avg_speed": deque(maxlen=window_size),
                "waiting_time": deque(maxlen=window_size),
                "stop_and_go_freq": deque(maxlen=window_size),
                "fuel_consumption": deque(maxlen=window_size),
                "co2_emissions": deque(maxlen=window_size),
                "queue_length": deque(maxlen=window_size),
                "occupancy": deque(maxlen=window_size)
            } for edge in connected_edges
        }

    def update_edge_data(self, edge, data_point):
        """
        data_point is a dict with the current timestep's aggregated values for the edge.
        """
        if edge in self.edge_data:
            for key in self.edge_data[edge]:
                self.edge_data[edge][key].append(data_point.get(key, 0.0))

    def get_average_stats(self, edge):
        """
        Returns the average of the metrics over the rolling window for a specific edge.
        """
        if edge not in self.edge_data:
            return {}
            
        stats = {}
        for key, queue in self.edge_data[edge].items():
            if len(queue) > 0:
                stats[key] = np.mean(queue)
            else:
                stats[key] = 0.0
        return stats
