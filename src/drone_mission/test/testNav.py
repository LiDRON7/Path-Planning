"""
Tests for Navigator ROS2 node.

Run with:
    pytest testNav.py -v


"""

import math
import pytest
from unittest.mock import MagicMock, patch, call

# ---------------------------------------------------------------------------
# Helpers – let tests import without a live ROS2 environment
# ---------------------------------------------------------------------------

import sys

# Stub out rclpy and px4_msgs so we can import Navigator in a plain Python env
rclpy_stub = MagicMock()
sys.modules.setdefault("rclpy", rclpy_stub)
sys.modules.setdefault("rclpy.node", rclpy_stub)
sys.modules.setdefault("px4_msgs", MagicMock())
sys.modules.setdefault("px4_msgs.msg", MagicMock())
sys.modules.setdefault("sensor_msgs", MagicMock())
sys.modules.setdefault("sensor_msgs.msg", MagicMock())

# Minimal BaseDrone stub so Navigator.__init__ doesn't need a live node
class _BaseDroneStub:
    def __init__(self, name):
        self._name = name
        self.trajectory_setpoint_pub = MagicMock()

    def create_timer(self, period, cb):
        return MagicMock()

    def create_subscription(self, *args, **kwargs):
        return MagicMock()

    def get_logger(self):
        logger = MagicMock()
        logger.info = MagicMock()
        logger.warn = MagicMock()
        return logger

    def get_clock(self):
        clock = MagicMock()
        clock.now.return_value.nanoseconds = 0
        return clock

    def publish_offboard_control_mode(self):
        pass

    def arm(self):
        pass

    def engage_offboard_mode(self):
        pass


# Patch BaseDrone before importing Navigator
with patch.dict("sys.modules", {"drone_pkg.base_drone": MagicMock()}):
    import importlib, types

    # Build a fake module that exports our stub
    fake_base = types.ModuleType("drone_pkg.base_drone")
    fake_base.BaseDrone = _BaseDroneStub
    sys.modules["drone_pkg.base_drone"] = fake_base

    # Now import (adjust the import path to match your package layout)
    # If your package is called drone_pkg and the file is navigator.py:
    #   from drone_pkg.navigator import Navigator
    # For the test we reconstruct the class inline so the file can be run
    # independently of the installed package.  Replace the exec block below
    # with a real import once your package is installed.

    import textwrap, pathlib

    _SOURCE = pathlib.Path(__file__).parent / "navigator.py"
    if _SOURCE.exists():
        _ns = {"BaseDrone": _BaseDroneStub}
        exec(compile(_SOURCE.read_text().replace(
            "from .base_drone import BaseDrone",
            ""  # already injected via _ns
        ), str(_SOURCE), "exec"), _ns)
        Navigator = _ns["Navigator"]
    else:
        # Inline minimal reproduction for standalone testing
        class Navigator(_BaseDroneStub):
            def __init__(self):
                super().__init__("navigator_node")
                self.timer = self.create_timer(0.1, self.timer_callback)
                self.target_altitude = -5.0
                self.mission_started = False
                self.gps_subscriber = self.create_subscription(None, "/mavros/global_position/global", self.gps_callback, 10)
                self.current_gps = None
                self.home_lat = None
                self.home_lon = None
                self.target_lat = 18.2100
                self.target_lon = -67.1400
                self.target_x = None
                self.target_y = None
                self.arrival_tolerance = 2.0

            def gps_callback(self, msg):
                self.current_gps = msg
                if self.home_lat is None:
                    self.home_lat = msg.latitude
                    self.home_lon = msg.longitude
                    self.get_logger().info("Home position set")

            def distance_meters(self, lat1, lon1, lat2, lon2):
                R = 6371000
                phi1, phi2 = math.radians(lat1), math.radians(lat2)
                dphi = math.radians(lat2 - lat1)
                dlambda = math.radians(lon2 - lon1)
                a = (math.sin(dphi / 2) ** 2 +
                     math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2)
                c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
                return R * c

            def gps_to_ned(self, lat, lon):
                R = 6378137.0
                d_lat = math.radians(lat - self.home_lat)
                d_lon = math.radians(lon - self.home_lon)
                x = R * d_lon * math.cos(math.radians(self.home_lat))
                y = R * d_lat
                return x, y

            def navigate_to_waypoint(self):
                current_x, current_y = self.gps_to_ned(
                    self.current_gps.latitude, self.current_gps.longitude
                )
                error_x = self.target_x - current_x
                error_y = self.target_y - current_y
                distance = math.sqrt(error_x ** 2 + error_y ** 2)
                if distance < self.arrival_tolerance:
                    self.get_logger().info("Waypoint reached ✅")
                    return
                direction_x = error_x / distance
                direction_y = error_y / distance
                speed = 1.5
                step_x = current_x + direction_x * speed
                step_y = current_y + direction_y * speed
                from unittest.mock import MagicMock
                msg = MagicMock()
                msg.position = [step_x, step_y, self.target_altitude]
                msg.yaw = math.atan2(direction_y, direction_x)
                msg.timestamp = 0
                self.trajectory_setpoint_pub.publish(msg)

            def timer_callback(self):
                self.publish_offboard_control_mode()
                if self.current_gps is None or self.home_lat is None:
                    self.get_logger().warn("Waiting for GPS...")
                    return
                if self.target_x is None:
                    self.target_x, self.target_y = self.gps_to_ned(
                        self.target_lat, self.target_lon
                    )
                    self.get_logger().info("Target converted to NED")
                self.navigate_to_waypoint()
                if not self.mission_started:
                    self.arm()
                    self.engage_offboard_mode()
                    self.mission_started = True


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def nav():
    """Fresh Navigator instance for each test."""
    n = Navigator()
    # Inject a home position so math tests don't need to go through gps_callback
    n.home_lat = 18.2000
    n.home_lon = -67.1500
    return n


def _make_gps(lat, lon, alt=10.0):
    msg = MagicMock()
    msg.latitude = lat
    msg.longitude = lon
    msg.altitude = alt
    return msg


# ===========================================================================
# Layer 1 – pure math (no ROS2, no state side-effects)
# ===========================================================================

class TestDistanceMeters:
    def test_same_point_is_zero(self, nav):
        assert nav.distance_meters(18.2, -67.1, 18.2, -67.1) == pytest.approx(0.0)

    def test_known_distance_north(self, nav):
        # 0.01° latitude ≈ 1111 m anywhere on Earth
        d = nav.distance_meters(18.2, -67.1, 18.21, -67.1)
        assert 1100 < d < 1120, f"Expected ~1111 m, got {d:.1f} m"

    def test_known_distance_east(self, nav):
        # 0.01° longitude at lat 18° ≈ 1055 m
        d = nav.distance_meters(18.2, -67.1, 18.2, -67.09)
        assert 1040 < d < 1070, f"Expected ~1055 m, got {d:.1f} m"

    def test_symmetry(self, nav):
        d1 = nav.distance_meters(18.2, -67.1, 18.25, -67.05)
        d2 = nav.distance_meters(18.25, -67.05, 18.2, -67.1)
        assert d1 == pytest.approx(d2, rel=1e-9)

    def test_large_distance(self, nav):
        # Mayaguez → San Juan ≈ 113 km
        d = nav.distance_meters(18.2013, -67.1397, 18.4655, -66.1057)
        assert 110_000 < d < 116_000 

class TestGpsToNed:
    def test_origin_maps_to_zero(self, nav):
        x, y = nav.gps_to_ned(nav.home_lat, nav.home_lon)
        assert x == pytest.approx(0.0, abs=1e-6)
        assert y == pytest.approx(0.0, abs=1e-6)

    def test_north_is_positive_y(self, nav):
        _, y = nav.gps_to_ned(nav.home_lat + 0.001, nav.home_lon)
        assert y > 0, "Moving north should give positive y in NED"

    def test_east_is_positive_x(self, nav):
        x, _ = nav.gps_to_ned(nav.home_lat, nav.home_lon + 0.001)
        assert x > 0, "Moving east should give positive x in NED"

    def test_scale_approximately_correct(self, nav):
        # 0.001° lat ≈ 111 m
        _, y = nav.gps_to_ned(nav.home_lat + 0.001, nav.home_lon)
        assert 110 < y < 113, f"Expected ~111 m north, got {y:.2f} m"

    def test_west_is_negative_x(self, nav):
        x, _ = nav.gps_to_ned(nav.home_lat, nav.home_lon - 0.001)
        assert x < 0

    def test_south_is_negative_y(self, nav):
        _, y = nav.gps_to_ned(nav.home_lat - 0.001, nav.home_lon)
        assert y < 0

    def test_requires_home_set(self):
        """gps_to_ned must raise (or produce NaN) if home is None."""
        n = Navigator()
        # home_lat/lon are None → math.radians(None) raises TypeError
        with pytest.raises(TypeError):
            n.gps_to_ned(18.21, -67.14)


# ===========================================================================
# Layer 2 – state logic with mocked ROS2 internals
# ===========================================================================

class TestGpsCallback:
    def test_sets_current_gps(self, nav):
        msg = _make_gps(18.21, -67.14)
        nav.home_lat = None  # reset so callback also sets home
        nav.gps_callback(msg)
        assert nav.current_gps is msg

    def test_sets_home_on_first_call(self, nav):
        nav.home_lat = None
        nav.home_lon = None
        msg = _make_gps(18.21, -67.14)
        nav.gps_callback(msg)
        assert nav.home_lat == 18.21
        assert nav.home_lon == -67.14

    def test_home_not_overwritten_on_second_call(self, nav):
        nav.home_lat = None
        nav.home_lon = None
        nav.gps_callback(_make_gps(18.21, -67.14))
        nav.gps_callback(_make_gps(18.22, -67.13))
        assert nav.home_lat == 18.21, "Home must not drift on subsequent GPS fixes"
        assert nav.home_lon == -67.14


class TestTimerCallback:
    def test_warns_when_no_gps(self, nav):
        nav.current_gps = None
        nav.home_lat = None
        logger = MagicMock()
        nav.get_logger = lambda: logger
        nav.timer_callback()
        logger.warn.assert_called()

    def test_converts_target_once(self, nav):
        nav.current_gps = _make_gps(nav.home_lat, nav.home_lon)
        nav.target_x = None
        nav.timer_callback()
        assert nav.target_x is not None
        tx_first = nav.target_x
        nav.timer_callback()
        assert nav.target_x == tx_first, "Target NED should be computed only once"

    def test_arms_and_engages_once(self, nav):
        nav.arm = MagicMock()
        nav.engage_offboard_mode = MagicMock()
        nav.current_gps = _make_gps(nav.home_lat, nav.home_lon)
        nav.timer_callback()
        nav.timer_callback()
        nav.arm.assert_called_once()
        nav.engage_offboard_mode.assert_called_once()

    def test_does_not_arm_before_gps(self, nav):
        """BUG GUARD: arm() must not fire before GPS is ready."""
        nav.arm = MagicMock()
        nav.current_gps = None
        nav.timer_callback()
        nav.arm.assert_not_called()


class TestNavigateToWaypoint:
    def _set_position(self, nav, lat, lon):
        nav.current_gps = _make_gps(lat, lon)
        if nav.target_x is None:
            nav.target_x, nav.target_y = nav.gps_to_ned(nav.target_lat, nav.target_lon)

    def test_publishes_setpoint_when_far(self, nav):
        self._set_position(nav, nav.home_lat, nav.home_lon)
        nav.navigate_to_waypoint()
        nav.trajectory_setpoint_pub.publish.assert_called_once()

    def test_does_not_publish_at_waypoint(self, nav):
        # Place drone exactly on the target
        nav.current_gps = _make_gps(nav.target_lat, nav.target_lon)
        nav.target_x, nav.target_y = nav.gps_to_ned(nav.target_lat, nav.target_lon)
        nav.navigate_to_waypoint()
        nav.trajectory_setpoint_pub.publish.assert_not_called()

    def test_altitude_is_constant(self, nav):
        self._set_position(nav, nav.home_lat, nav.home_lon)
        nav.navigate_to_waypoint()
        published_msg = nav.trajectory_setpoint_pub.publish.call_args[0][0]
        assert published_msg.position[2] == nav.target_altitude

    def test_step_does_not_overshoot(self, nav):
        """
        BUG: step = current + direction * speed moves in ABSOLUTE coords but
        the drone is NOT at origin, so the step overshoots.  This test
        documents the correct behavior: the commanded position should be
        (current + one speed-step toward target), not (origin + step).
        """
        # Place drone 10 m south of home, target 100 m north of home
        lat_offset = 10 / 111_320  # ~10 m in degrees
        nav.current_gps = _make_gps(nav.home_lat - lat_offset, nav.home_lon)
        nav.target_x, nav.target_y = 0.0, 100.0  # 100 m north in NED
        current_x, current_y = nav.gps_to_ned(nav.current_gps.latitude, nav.current_gps.longitude)

        nav.navigate_to_waypoint()

        published_msg = nav.trajectory_setpoint_pub.publish.call_args[0][0]
        cmd_x, cmd_y, cmd_z = published_msg.position

        # The commanded position must be near current + one step, NOT near origin + step
        expected_y = current_y + 1.5  # speed = 1.5 m/s, heading north
        assert cmd_y == pytest.approx(expected_y, abs=0.1), (
            f"Overshoot bug: expected y≈{expected_y:.2f}, got {cmd_y:.2f}. "
            "Check that step_x/y uses absolute NED coords correctly."
        )

    def test_yaw_points_toward_target(self, nav):
        # Drone is south of target → yaw should be ~π/2 (north, in math convention)
        nav.current_gps = _make_gps(nav.home_lat, nav.home_lon)
        nav.target_x, nav.target_y = 0.0, 500.0  # due north
        nav.navigate_to_waypoint()
        msg = nav.trajectory_setpoint_pub.publish.call_args[0][0]
        assert msg.yaw == pytest.approx(math.pi / 2, abs=0.05)

    def test_yaw_within_valid_range(self, nav):
        """Yaw must stay within (-π, π]."""
        # Test several headings
        for bearing_deg in range(0, 360, 45):
            nav.trajectory_setpoint_pub.reset_mock()
            r = 500.0  # 500 m away
            nav.target_x = r * math.sin(math.radians(bearing_deg))
            nav.target_y = r * math.cos(math.radians(bearing_deg))
            nav.current_gps = _make_gps(nav.home_lat, nav.home_lon)
            nav.navigate_to_waypoint()
            msg = nav.trajectory_setpoint_pub.publish.call_args[0][0]
            assert -math.pi <= msg.yaw <= math.pi, (
                f"Yaw {msg.yaw:.4f} rad out of range at bearing {bearing_deg}°"
            )

    def test_arrival_within_tolerance(self, nav):
        """Drone within tolerance should NOT publish a new setpoint."""
        # 1.5 m from target (< 2.0 m tolerance)
        small_offset = 1.5 / 111_320
        nav.current_gps = _make_gps(nav.target_lat + small_offset, nav.target_lon)
        nav.target_x, nav.target_y = nav.gps_to_ned(nav.target_lat, nav.target_lon)
        nav.navigate_to_waypoint()
        nav.trajectory_setpoint_pub.publish.assert_not_called()

    def test_arrival_outside_tolerance(self, nav):
        """Drone outside tolerance SHOULD publish a setpoint."""
        far_offset = 10.0 / 111_320  # 10 m away
        nav.current_gps = _make_gps(nav.target_lat + far_offset, nav.target_lon)
        nav.target_x, nav.target_y = nav.gps_to_ned(nav.target_lat, nav.target_lon)
        nav.navigate_to_waypoint()
        nav.trajectory_setpoint_pub.publish.assert_called_once()

