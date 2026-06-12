import logging

class RoadConditionManager:
    def __init__(self, degraded_edges, mode, activate_time, v_low=5.0, v_high=12.0, period_s=20.0, event_duration=600.0, dt=1.0):
        """
        Manages mid-simulation physical road degradations.
        
        Args:
            degraded_edges (list): List of edge IDs to degrade.
            mode (str): 'none', 'rough', or 'accident'.
            activate_time (float): Simulation time (seconds) to begin degradation.
            v_low (float): Lower bound for speed oscillation (m/s).
            v_high (float): Upper bound for speed oscillation (m/s).
            period_s (float): Oscillation period (seconds).
            event_duration (float): Duration for 'accident' mode blockages.
            dt (float): Simulation step size.
        """
        self.degraded_edges = degraded_edges
        self.mode = mode
        self.activate_time = activate_time
        self.v_low = v_low
        self.v_high = v_high
        self.period_s = period_s
        self.event_duration = event_duration
        self.dt = dt

        self.active = False
        self.restored = False
        self.original_speeds = {}
        self.original_permissions = {}
        self.blocked_lanes = []
        self.last_toggle_time = 0.0
        self.current_oscillation_state = "low" # Start by dropping to v_low

        # Dependency injection for TraCI and calc to allow mocking in unit tests
        self._traci = None
        self._calc = None

    def bind(self, traci_module, calc_module):
        """Bind the actual traci and EdgeCostCalculator instances for execution."""
        self._traci = traci_module
        self._calc = calc_module

    def step(self, sim_time):
        """Called once per simulation step."""
        if self.mode == "none" or len(self.degraded_edges) == 0:
            return

        # Check for initial activation
        if not self.active and not self.restored and sim_time >= self.activate_time:
            self._activate()

        # Handle active state mechanics
        if self.active:
            if self.mode == "rough":
                self._oscillate_rough(sim_time)
            elif self.mode == "accident":
                # Check for restoration
                if sim_time >= self.activate_time + self.event_duration:
                    self.deactivate()

    def _verify_baseline_locks(self):
        """Enforce that baselines for degraded edges are locked before activation."""
        if not self._calc:
            return
            
        unlocked = []
        for edge in self.degraded_edges:
            if edge not in self._calc._fuel_baseline:
                unlocked.append(edge)
                
        if unlocked:
            logging.warning(
                f"WARNING: RoadConditionManager activating at {self.activate_time}s but baselines "
                f"are UNLOCKED for edges {unlocked}. The RSU will be blind to the extra fuel "
                f"on these edges because it will absorb the degraded rate into the baseline."
            )
        else:
            logging.info(f"RoadConditionManager: baselines locked for {len(self.degraded_edges)}/{len(self.degraded_edges)} degraded edges.")

    def _activate(self):
        """Activates the degradation."""
        self.active = True
        self._verify_baseline_locks()

        if self.mode == "rough":
            logging.info(f"RoadConditionManager: Activating ROUGH mode on edges {self.degraded_edges}")
            for edge in self.degraded_edges:
                try:
                    # In traci, edge.getMaxSpeed returns the max speed of the edge
                    orig_speed = self._traci.lane.getMaxSpeed(edge + '_0')
                    self.original_speeds[edge] = orig_speed
                    # Apply initial drop
                    for i in range(self._traci.edge.getLaneNumber(edge)):\n                        self._traci.lane.setMaxSpeed(edge + '_' + str(i), self.v_low)
                except Exception as e:
                    logging.warning(f"Failed to activate rough mode on {edge}: {e}")
            self.last_toggle_time = self.activate_time
            self.current_oscillation_state = "low"

        elif self.mode == "accident":
            logging.info(f"RoadConditionManager: Activating ACCIDENT mode on edges {self.degraded_edges}")
            for edge in self.degraded_edges:
                try:
                    # Get lanes for this edge
                    lane_count = self._traci.edge.getLaneNumber(edge)
                    if lane_count > 1:
                        # Prefer blocking lane 0, leaving other lanes open
                        lane_id = f"{edge}_0"
                        orig_permissions = self._traci.lane.getAllowed(lane_id)
                        self.original_permissions[lane_id] = orig_permissions
                        # Disallow passenger vehicles to block it
                        self._traci.lane.setDisallowed(lane_id, ["passenger", "custom1", "custom2"])
                        self.blocked_lanes.append(lane_id)
                    else:
                        logging.warning(f"Edge {edge} is single-lane. Blocking it fully will force travel-time rerouting, nullifying the experiment.")
                        # Still block it as requested, but log the warning
                        lane_id = f"{edge}_0"
                        orig_permissions = self._traci.lane.getAllowed(lane_id)
                        self.original_permissions[lane_id] = orig_permissions
                        self._traci.lane.setDisallowed(lane_id, ["passenger", "custom1", "custom2"])
                        self.blocked_lanes.append(lane_id)
                except Exception as e:
                    logging.warning(f"Failed to activate accident mode on {edge}: {e}")

    def _oscillate_rough(self, sim_time):
        """Toggles the speed limit between v_low and v_high every period_s."""
        if sim_time - self.last_toggle_time >= (self.period_s / 2.0):
            self.last_toggle_time = sim_time
            
            # Toggle state
            if self.current_oscillation_state == "low":
                self.current_oscillation_state = "high"
                target_v = self.v_high
            else:
                self.current_oscillation_state = "low"
                target_v = self.v_low
                
            for edge in self.degraded_edges:
                if edge in self.original_speeds:
                    # Safety check: don't raise v_high above original speed limit
                    safe_target = min(target_v, self.original_speeds[edge])
                    try:
                        for i in range(self._traci.edge.getLaneNumber(edge)):\n                            self._traci.lane.setMaxSpeed(edge + '_' + str(i), safe_target)
                    except:
                        pass

    def deactivate(self):
        """Restores the original physical state of the edges."""
        if not self.active:
            return
            
        logging.info("RoadConditionManager: Deactivating and restoring original conditions.")
        
        if self.mode == "rough":
            for edge, orig_v in self.original_speeds.items():
                try:
                    for i in range(self._traci.edge.getLaneNumber(edge)):\n                        self._traci.lane.setMaxSpeed(edge + '_' + str(i), orig_v)
                except:
                    pass
            self.original_speeds.clear()
            
        elif self.mode == "accident":
            for lane_id, allowed in self.original_permissions.items():
                try:
                    self._traci.lane.setAllowed(lane_id, allowed)
                except:
                    pass
            self.original_permissions.clear()
            self.blocked_lanes.clear()

        self.active = False
        self.restored = True
