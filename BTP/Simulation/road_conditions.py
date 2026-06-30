import logging


class RoadConditionManager:
    def __init__(self, degraded_edges, mode, activate_time,
                 v_low=5.0, v_high=12.0, period_s=20.0, event_duration=600.0, dt=1.0,
                 grade_emission_class="HBEFA3/PC_G_EU0"):
        """
        Manages mid-simulation physical road degradations.

        Args:
            degraded_edges (list): List of edge IDs to degrade.
            mode (str): 'none', 'rough', 'accident', or 'grade'.
            activate_time (float): Simulation time (seconds) to begin degradation.
            v_low (float): Lower bound for speed oscillation (m/s).
            v_high (float): Upper bound for speed oscillation (m/s).
            period_s (float): Oscillation period (seconds).
            event_duration (float): Duration for 'accident' mode blockages.
            dt (float): Simulation step size.
            grade_emission_class (str): Heavier HBEFA emission class applied in grade mode.

        Mode regimes:
            rough    - Oscillates speed limit between v_low and v_high every period_s/2.
                       Raises fuel THROUGH slowing (time-visible regime).
                       Tests "congestion already shows in time" hypothesis.
            accident - Blocks lane 0 of multi-lane edges, leaving remaining lanes passable.
                       Creates queue + merge stop-and-go without full closure.
                       Raises fuel THROUGH queueing (partially time-visible).
            grade    - Swaps each vehicle's emission class to a heavier HBEFA3 class on
                       entry to degraded edges, restoring on exit. Speed is NOT changed.
                       Target: fuel/m up >= 30%, traversal time up <= ~5%.
                       Tests the "fuel-costly but not slow" decoupled hypothesis.
        """
        self.degraded_edges = degraded_edges
        self.mode = mode
        self.activate_time = activate_time
        self.v_low = v_low
        self.v_high = v_high
        self.period_s = period_s
        self.event_duration = event_duration
        self.dt = dt
        self.grade_emission_class = grade_emission_class

        self.active = False
        self.restored = False
        self.original_speeds = {}       # edge_id -> original speed (m/s)
        self.original_permissions = {}  # lane_id -> original allowed list
        self.blocked_lanes = []
        self.last_toggle_time = 0.0
        self.current_oscillation_state = "low"  # Start by dropping to v_low

        # grade mode: per-vehicle original emission class, restored on edge exit
        self._grade_modified_vehicles = {}  # vid -> original_emission_class

        # Dependency injection for TraCI, calc, and RSU manager
        self._traci = None
        self._calc = None
        self._rsu_manager = None

    def bind(self, traci_module, calc_module, rsu_manager=None):
        """Bind the actual traci, EdgeCostCalculator, and RSUManager instances."""
        self._traci = traci_module
        self._calc = calc_module
        self._rsu_manager = rsu_manager

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
                # Check for restoration after event_duration
                if sim_time >= self.activate_time + self.event_duration:
                    self.deactivate()
            elif self.mode == "grade":
                self._apply_grade()

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
            logging.info(
                f"RoadConditionManager: baselines locked for "
                f"{len(self.degraded_edges)}/{len(self.degraded_edges)} degraded edges."
            )

    def _activate(self):
        """Activates the degradation."""
        self.active = True
        self._verify_baseline_locks()

        if self.mode == "rough":
            logging.info(
                f"RoadConditionManager: Activating ROUGH mode on edges {self.degraded_edges}"
            )
            for edge in self.degraded_edges:
                try:
                    # Read original speed from lane 0 (lane API, not edge API)
                    orig_speed = self._traci.lane.getMaxSpeed(edge + "_0")
                    self.original_speeds[edge] = orig_speed
                    # Drop every lane to v_low
                    n_lanes = self._traci.edge.getLaneNumber(edge)
                    for i in range(n_lanes):
                        self._traci.lane.setMaxSpeed(edge + "_" + str(i), self.v_low)
                except Exception as e:
                    logging.warning(f"Failed to activate rough mode on {edge}: {e}")
            self.last_toggle_time = self.activate_time
            self.current_oscillation_state = "low"

        elif self.mode == "accident":
            logging.info(
                f"RoadConditionManager: Activating ACCIDENT mode on edges {self.degraded_edges}"
            )
            for edge in self.degraded_edges:
                try:
                    lane_count = self._traci.edge.getLaneNumber(edge)
                    if lane_count > 1:
                        # Block lane 0 only, leaving the remaining lane(s) passable.
                        # This creates queue + merge stop-and-go without a full closure.
                        lane_id = f"{edge}_0"
                        orig_permissions = self._traci.lane.getAllowed(lane_id)
                        self.original_permissions[lane_id] = orig_permissions
                        self._traci.lane.setDisallowed(
                            lane_id, ["passenger", "custom1", "custom2"]
                        )
                        self.blocked_lanes.append(lane_id)
                    else:
                        logging.warning(
                            f"Edge {edge} is single-lane. Blocking it fully will force "
                            f"travel-time rerouting by SUMO, nullifying the experiment."
                        )
                        lane_id = f"{edge}_0"
                        orig_permissions = self._traci.lane.getAllowed(lane_id)
                        self.original_permissions[lane_id] = orig_permissions
                        self._traci.lane.setDisallowed(
                            lane_id, ["passenger", "custom1", "custom2"]
                        )
                        self.blocked_lanes.append(lane_id)
                except Exception as e:
                    logging.warning(f"Failed to activate accident mode on {edge}: {e}")

        elif self.mode == "grade":
            logging.info(
                f"RoadConditionManager: Activating GRADE mode on edges {self.degraded_edges} "
                f"(emission class: {self.grade_emission_class}). Speed limits unchanged."
            )
            import sys as _sys
            _sys.stderr.write(
                f"[GRADE_ACTIVATE] edges={self.degraded_edges} "
                f"activate_time={self.activate_time} "
                f"emission_class={self.grade_emission_class}\n"
            )
            _sys.stderr.flush()
            # Freeze the fuel baseline for each degraded edge so the pre-activation
            # free-flow floor (EU4 rate) is preserved as the F denominator. Without
            # this the EMA absorbs the elevated EU0 departure rates and F collapses
            # to ~0 before any ego trip runs.
            if self._calc is not None:
                for edge in self.degraded_edges:
                    self._calc.freeze_baseline(edge)
            # Clear the fuel/CO2 rolling windows so pre-grade EU4 trip data does
            # not dilute the EU0 signal.  On low-traffic corridors the 60-step
            # window rarely captures a departure; leftover EU4 entries would bias
            # fuel_cons toward EU4 and suppress F even after EU0 grade is active.
            if self._rsu_manager is not None:
                self._rsu_manager.clear_fuel_deques(self.degraded_edges)
                import sys as _sys
                _sys.stderr.write(f"[FUEL_DEQUES_CLEARED] {self.degraded_edges}\n")
                _sys.stderr.flush()
            # No speed change on activation — per-vehicle emission class swap
            # is handled each step by _apply_grade().

    def _oscillate_rough(self, sim_time):
        """Toggles the speed limit between v_low and v_high every period_s / 2."""
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
                    # Safety: don't exceed the original speed limit
                    safe_target = min(target_v, self.original_speeds[edge])
                    try:
                        n_lanes = self._traci.edge.getLaneNumber(edge)
                        for i in range(n_lanes):
                            self._traci.lane.setMaxSpeed(
                                edge + "_" + str(i), safe_target
                            )
                    except Exception:
                        pass

    def _apply_grade(self):
        """
        Grade mode per-step handler.

        For each degraded edge, swap the emission class of vehicles currently on
        that edge to self.grade_emission_class. When a vehicle leaves the degraded
        edge set, restore its original emission class.

        This raises fuel/m WITHOUT touching speed limits, implementing the
        'fuel-costly but not slow' decoupled hypothesis.
        """
        # Gather all vehicle IDs currently on any degraded edge
        currently_on_degraded = set()
        for edge in self.degraded_edges:
            try:
                vids = self._traci.edge.getLastStepVehicleIDs(edge)
                currently_on_degraded.update(vids)
            except Exception:
                pass

        # Apply heavy emission class to newly-entered vehicles
        already_modified = set(self._grade_modified_vehicles.keys())
        newly_entered = currently_on_degraded - already_modified
        for vid in newly_entered:
            try:
                orig_class = self._traci.vehicle.getEmissionClass(vid)
                self._traci.vehicle.setEmissionClass(vid, self.grade_emission_class)
                self._grade_modified_vehicles[vid] = orig_class
            except Exception as e:
                logging.debug(f"[grade] Could not set emission class for {vid}: {e}")

        # Restore emission class for vehicles that have left the degraded edges
        exited = already_modified - currently_on_degraded
        for vid in exited:
            orig_class = self._grade_modified_vehicles.pop(vid)
            try:
                if vid in self._traci.vehicle.getIDList():
                    self._traci.vehicle.setEmissionClass(vid, orig_class)
            except Exception as e:
                logging.debug(f"[grade] Could not restore emission class for {vid}: {e}")

    def deactivate(self):
        """Restores the original physical state of the edges."""
        if not self.active:
            return

        logging.info("RoadConditionManager: Deactivating and restoring original conditions.")

        if self.mode == "rough":
            for edge, orig_v in self.original_speeds.items():
                try:
                    n_lanes = self._traci.edge.getLaneNumber(edge)
                    for i in range(n_lanes):
                        self._traci.lane.setMaxSpeed(edge + "_" + str(i), orig_v)
                except Exception:
                    pass
            self.original_speeds.clear()

        elif self.mode == "accident":
            for lane_id, allowed in self.original_permissions.items():
                try:
                    self._traci.lane.setAllowed(lane_id, allowed)
                except Exception:
                    pass
            self.original_permissions.clear()
            self.blocked_lanes.clear()

        elif self.mode == "grade":
            # Restore emission class for all still-modified vehicles
            for vid, orig_class in list(self._grade_modified_vehicles.items()):
                try:
                    if vid in self._traci.vehicle.getIDList():
                        self._traci.vehicle.setEmissionClass(vid, orig_class)
                except Exception:
                    pass
            self._grade_modified_vehicles.clear()
            # Unfreeze baselines so the EMA resumes tracking the restored EU4 rates.
            if self._calc is not None:
                for edge in self.degraded_edges:
                    self._calc.unfreeze_baseline(edge)

        self.active = False
        self.restored = True
