import rclpy
import math

from .base_drone import BaseDrone 
from px4_msgs.msg import TrajectorySetpoint, VehicleLocalPosition, VehicleCommand, VehicleStatus
from sensor_msgs.msg import NavSatFix
from .mission_state_machine import MissionStateMachine, MissionState


class Navigator(BaseDrone):
    def __init__(self):
        super().__init__('navigator_node')

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.target_altitude = -5.0
        self.mission_started = False

        # GPS Subscriber
        self.gps_subscriber = self.create_subscription(
            NavSatFix,
            '/gps',
            self.gps_callback,
            10
        
        )
          #--- QoS / extra subscriptions----
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Local position gives us the NED altitude we need for TAKEOFF check
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/gps',
            self.local_position_callback,
            qos_profile
        )

        self.current_altitude_ned = 0.0

        self.current_gps = None

        # Home reference
        self.home_lat = None
        self.home_lon = None

        # Target waypoint
        self.target_lat = 18.2100
        self.target_lon = -67.1400

        self.target_x = None
        self.target_y = None

        self.arrival_tolerance = 2.0
        self.sm = MissionStateMachine(
            logger=self.get_logger(),
            target_altitude=self.target_altitude,
            arrival_tolerance=self.arrival_tolerance,
        )

    def gps_callback(self, msg):
        self.current_gps = msg

        if self.home_lat is None:
            self.home_lat = msg.latitude
            self.home_lon = msg.longitude
            self.get_logger().info("Home position set")

    def local_position_callback(self, msg):
        self.current_altitude_ned = msg.z

    def gps_to_ned(self, lat, lon):
        R = 6378137.0

        d_lat = math.radians(lat - self.home_lat)
        d_lon = math.radians(lon - self.home_lon)

        x = R * d_lon * math.cos(math.radians(self.home_lat))
        y = R * d_lat

        return x, y

    # Main loop
    def timer_callback(self):
        # Always send the offboard heartbeat so PX4 stays in offboard mode
        self.publish_offboard_control_mode()

        # ── IDLE: wait for GPS, then arm + offboard and transition to TAKEOFF
        if self.sm.is_idle():
            if self.current_gps is None or self.home_lat is None:
                self.get_logger().warn("Waiting for GPS...", throttle_duration_sec=2.0)
                return

            # Pre-compute NED target once home is known
            if self.target_x is None:
                self.target_x, self.target_y = self.gps_to_ned(
                    self.target_lat, self.target_lon
                )
                self.get_logger().info("Target converted to NED")

            self.arm()
            self.engage_offboard_mode()
            self.sm.on_gps_ready()   # IDLE → TAKEOFF
            return
        
        # ── TAKEOFF: command the target altitude and wait until reached
        if self.sm.is_takeoff():
            self._publish_hold_with_altitude(self.target_altitude)

            if self.sm.check_takeoff_complete(self.current_altitude_ned):
                pass   # state machine logs the transition
            return

        # ── NAVIGATE: fly toward the waypoint
        if self.sm.is_navigate():
            self._navigate_to_waypoint()
            return

        # ── LAND: send land command once, then poll arm-state for completion
        if self.sm.is_land():
            self._execute_land()
            return



    def navigate_to_waypoint(self):
        current_x, current_y = self.gps_to_ned(
            self.current_gps.latitude,
            self.current_gps.longitude
        )

        error_x = self.target_x - current_x
        error_y = self.target_y - current_y

        distance = math.sqrt(error_x**2 + error_y**2)

        self.get_logger().info(f"Distance: {distance:.2f} m")

        if self.sm.check_waypoint_reached(distance):
            return

        # Prevent division by zero
        if distance < 0.001:
            return

        direction_x = error_x / distance
        direction_y = error_y / distance

        speed = 1.5

        step_x = current_x + direction_x * speed
        step_y = current_y + direction_y * speed

        msg = TrajectorySetpoint()
        msg.position = [step_x, step_y, self.target_altitude]
        msg.yaw = math.atan2(direction_y, direction_x)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        self.trajectory_setpoint_pub.publish(msg)

    def publish_hold_position(self, altitude):
        current_x, current_y = self.gps_to_ned(
            self.current_gps.latitude,
            self.current_gps.longitude
        )

        msg = TrajectorySetpoint()
        msg.position = [current_x, current_y, altitude]
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        self.trajectory_setpoint_pub.publish(msg)
    
    self._land_command_sent = False

    def execute_land(self):
        if not self._land_command_sent:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.get_logger().info("Landing...")
            self._land_command_sent = True

        if self.arm_state == VehicleStatus.ARMING_STATE_DISARMED:
            self.sm.on_landed()
            self.get_logger().info("Mission complete")
            
def main(args=None):
    rclpy.init(args=args)
    navigator = Navigator()
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info("User interrupted. Shutting down.")
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()