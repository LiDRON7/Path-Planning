from enum import Enum, auto


class MissionState(Enum):
    IDLE      = auto()
    TAKEOFF   = auto()
    NAVIGATE  = auto()
    LAND      = auto()


class MissionStateMachine:
    """
    Event-driven state machine that manages the drone's mission phases.

    Transitions
    -----------
    IDLE      --[gps_ready]--> TAKEOFF
    TAKEOFF   --[altitude_reached]--> NAVIGATE
    NAVIGATE  --[waypoint_reached]--> LAND
    LAND      --[landed]--> IDLE  (terminal for a single-waypoint mission)
    """

    def __init__(self, logger, target_altitude: float, arrival_tolerance: float = 2.0):
        """
        Parameters
        ----------
        logger           : rclpy logger (node.get_logger())
        target_altitude  : cruise altitude in NED metres (negative = up, e.g. -5.0)
        arrival_tolerance: horizontal distance threshold in metres to consider a
                           waypoint reached.
        """
        self._state              = MissionState.IDLE
        self._logger             = logger
        self.target_altitude     = target_altitude          # NED (negative = up)
        self.arrival_tolerance   = arrival_tolerance
        self.altitude_tolerance  = 0.5                      # metres

    # Public interface

    @property
    def state(self) -> MissionState:
        return self._state

    def is_idle(self)     -> bool: return self._state == MissionState.IDLE
    def is_takeoff(self)  -> bool: return self._state == MissionState.TAKEOFF
    def is_navigate(self) -> bool: return self._state == MissionState.NAVIGATE
    def is_land(self)     -> bool: return self._state == MissionState.LAND

    # Transition triggers  (called from the timer callback)

    def on_gps_ready(self) -> bool:
        """
        Call once GPS and home reference are available.
        IDLE → TAKEOFF
        Returns True if a transition occurred.
        """
        if self._state == MissionState.IDLE:
            self._transition(MissionState.TAKEOFF)
            return True
        return False

    def check_takeoff_complete(self, current_altitude_ned: float) -> bool:
        """
        Call every tick while in TAKEOFF.
        current_altitude_ned: vehicle's local Z position in NED (negative = up).
        TAKEOFF → NAVIGATE when the drone is within altitude_tolerance of target.
        Returns True if a transition occurred.
        """
        if self._state != MissionState.TAKEOFF:
            return False

        # Both values are negative, "reached" means drone is at least as high
        # as the target (i.e. NED Z ≤ target, accounting for tolerance).
        if current_altitude_ned <= self.target_altitude + self.altitude_tolerance:
            self._transition(MissionState.NAVIGATE)
            return True
        return False

    def check_waypoint_reached(self, distance_m: float) -> bool:
        """
        Call every tick while in NAVIGATE.
        distance_m: horizontal distance to the waypoint in metres.
        NAVIGATE → LAND when distance < arrival_tolerance.
        Returns True if a transition occurred.
        """
        if self._state != MissionState.NAVIGATE:
            return False

        if distance_m < self.arrival_tolerance:
            self._transition(MissionState.LAND)
            return True
        return False

    def on_landed(self) -> bool:
        """
        Call when the landing sequence is complete (e.g. armed state changes
        to DISARMED or a landed flag is set).
        LAND → IDLE
        Returns True if a transition occurred.
        """
        if self._state == MissionState.LAND:
            self._transition(MissionState.IDLE)
            return True
        return False

    # Internal helpers

    def _transition(self, new_state: MissionState):
        self._logger.info(
            f"[StateMachine] {self._state.name} → {new_state.name}"
        )
        self._state = new_state
