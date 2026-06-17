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
                "vehicle_count":    deque(maxlen=window_size),
                "avg_speed":        deque(maxlen=window_size),
                "waiting_time":     deque(maxlen=window_size),
                "stop_and_go_freq": deque(maxlen=window_size),
                "fuel_consumption": deque(maxlen=window_size),
                "co2_emissions":    deque(maxlen=window_size),
                "queue_length":     deque(maxlen=window_size),
                "occupancy":        deque(maxlen=window_size),
            } for edge in connected_edges
        }

    def update_edge_data(self, edge, data_point):
        """
        data_point is a dict with the current timestep's aggregated values for the edge.
        """
        if edge in self.edge_data:
            for key in self.edge_data[edge]:
                val = data_point.get(key, 0.0)
                # Fuel/CO2 are per-trip aggregates (non-zero only when vehicles
                # depart this step).  Skip zeros so the deque stores only real
                # trip measurements — on low-traffic corridors this prevents the
                # window from being flushed to all-zeros between rare departures.
                if key in ("fuel_consumption", "co2_emissions") and val == 0.0:
                    continue
                self.edge_data[edge][key].append(val)

    def get_average_stats(self, edge):
        """
        Returns the average of the metrics over the rolling window for a specific edge.

        Fuel and CO2 metrics use a non-zero filter because the rolling window
        receives a zero entry on every step where no vehicle departs (empty-road
        or non-jam steps).  Averaging those zeros into the fuel mean produces a
        heavily diluted value (< actual fuel / trip × departure_rate / update_rate)
        that keeps F near zero even during grade degradation.  Averaging only the
        non-zero entries recovers the true mean-fuel-per-trip that _decompose
        expects, without changing the semantics of the other stats (speed,
        occupancy, etc.) which are correctly averaged over all steps.
        """
        if edge not in self.edge_data:
            return {}

        stats = {}
        for key, queue in self.edge_data[edge].items():
            if len(queue) > 0:
                if key in ("fuel_consumption", "co2_emissions"):
                    nonzero = [x for x in queue if x > 0]
                    stats[key] = float(np.mean(nonzero)) if nonzero else 0.0
                else:
                    stats[key] = float(np.mean(queue))
            else:
                stats[key] = 0.0
        return stats

    def clear(self):
        """Empty every rolling-window deque (called after a TraCI loadState, whose
        new background state makes the old observations meaningless).

        When ``traci.simulation.loadState()`` is called the entire simulation is
        atomically reset to the saved snapshot -- a completely different set of
        vehicles, speeds, queue lengths, and RNG state.  Any data still sitting in
        the ``edge_data`` deques was accumulated for a traffic situation that NO
        LONGER EXISTS in the newly-loaded state.  Continuing to average that stale
        data into the dynamic weights computed for the new state would give the
        router a distorted picture of the current network, especially for the
        "ours" arm that depends on accurate per-edge costs.

        The per-trip re-warm (``rewarm_steps`` background-only steps that follow
        every ``loadState`` in ``Simulation.run_checkpointed_campaign``)
        repopulates the deques from scratch using the freshly-loaded traffic, so
        the router has correct, current information before the ego is injected.

        EdgeCostCalculator free-flow fuel baselines (``_fuel_baseline``) are NOT
        cleared here; that happens between seeds via ``EdgeCostCalculator.reset()``
        because the baseline is a property of the edge, not of one traffic moment.
        """
        for edge in self.edge_data:
            for q in self.edge_data[edge].values():
                q.clear()
