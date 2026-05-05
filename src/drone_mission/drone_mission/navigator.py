import rclpy
import math

from .base_drone import BaseDrone 
from px4_msgs.msg import TrajectorySetpoint
from sensor_msgs.msg import NavSatFix


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


    def gps_callback(self, msg):
        self.current_gps = msg

        if self.home_lat is None:
            self.home_lat = msg.latitude
            self.home_lon = msg.longitude
            self.get_logger().info("Home position set")


    def gps_to_ned(self, lat, lon):
        R = 6378137.0

        d_lat = math.radians(lat - self.home_lat)
        d_lon = math.radians(lon - self.home_lon)

        x = R * d_lon * math.cos(math.radians(self.home_lat))
        y = R * d_lat

        return x, y


    def timer_callback(self):
        self.publish_offboard_control_mode()

        # Wait for GPS
        if self.current_gps is None or self.home_lat is None:
            self.get_logger().warn("Waiting for GPS...")
            return

        # Convert target once
        if self.target_x is None:
            self.target_x, self.target_y = self.gps_to_ned(
                self.target_lat,
                self.target_lon
            )
            self.get_logger().info("Target converted to NED")

        # Navigation loop
        self.navigate_to_waypoint()

        # Arm + Offboard (once)
        if not self.mission_started:
            self.arm()
            self.engage_offboard_mode()
            self.mission_started = True


    def navigate_to_waypoint(self):
        current_x, current_y = self.gps_to_ned(
            self.current_gps.latitude,
            self.current_gps.longitude
        )

        error_x = self.target_x - current_x
        error_y = self.target_y - current_y

        distance = math.sqrt(error_x**2 + error_y**2)

        self.get_logger().info(f"Distance: {distance:.2f} m")

        if distance < self.arrival_tolerance:
            self.get_logger().info("Waypoint reached")
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