import rclpy
import math

from .base_drone import BaseDrone 
from px4_msgs.msg import TrajectorySetpoint
from sensor_msgs.msg import NavSatFix

class Navigator(BaseDrone):
    def __init__(self):
        super().__init__('navigator_node') # node name

        # Setup a Timer
        # PX4 REQUIRES at least 2Hz of data to stay in Offboard mode.
        self.timer = self.create_timer(0.1, self.timer_callback)

        #  Define target
        #  NED coordinate system
        self.target_altitude = -5.0 
        self.mission_started = False
        # GPS Subscriber (FROM PX4 via bridge or sim) px4_msgs/msg/VehicleGpsPosition
        self.gps_subscriber = self.create_subscription(NavSatFix,'/gps',self.gps_callback,10)

        self.current_gps = None

        # Reference GPS (home position)
        self.home_lat = None
        self.home_lon = None

        # Example waypoint (lat/lon)
        self.target_lat = 18.2100
        self.target_lon = -67.1400

        self.target_x = None
        self.target_y = None

        # Tolerance in meters
        self.arrival_tolerance = 2.0

    def gps_callback(self, msg):
        self.current_gps = msg

        # Set home reference once
        if self.home_lat is None:
            self.home_lat = msg.latitude
            self.home_lon = msg.longitude
            self.get_logger().info("Home position set")

    def distance_meters(self, lat1, lon1, lat2, lon2):
        R = 6371000  # Earth radius in meters

        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)

        a = (math.sin(dphi / 2) ** 2 +
            math.cos(phi1) * math.cos(phi2) *
            math.sin(dlambda / 2) ** 2)

        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        return R * c
    def gps_to_ned(self, lat, lon):
        # converts coordinates from the GPS coordinate system into the Local NED (North-East-Down) frame.
        R = 6378137.0  

        d_lat = math.radians(lat - self.home_lat)
        d_lon = math.radians(lon - self.home_lon)

        x = R * d_lon * math.cos(math.radians(self.home_lat))  # East
        y = R * d_lat  # North

        return x, y
    
    def timer_callback(self):
        """runs every 0.1 seconds"""

        # Always send the Offboard Heartbeat
        self.publish_offboard_control_mode()

        # Wait until GPS is ready
        if self.current_gps is None or self.home_lat is None:
            self.get_logger().warn("Waiting for GPS...")
            return


        if self.target_x is None:
            self.target_x, self.target_y = self.gps_to_ned(
                self.target_lat,
                self.target_lon
            )
            self.get_logger().info("Target converted to NED")


        self.navigate_to_waypoint()

        # Arm + Offboard (unchanged)
        if not self.mission_started:
            self.arm()
            self.engage_offboard_mode()
            self.mission_started = True


    # def publish_trajectory_setpoint(self):
    #     """Send drone"""
    #     msg = TrajectorySetpoint()
    #     msg.yaw = 0.0 # Face North
    #     msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
    #     self.trajectory_setpoint_pub.publish(msg)

    def navigate_to_waypoint(self):

        # Current position in NED
        current_x, current_y = self.gps_to_ned(
            self.current_gps.latitude,
            self.current_gps.longitude
        )

        # Error vector
        error_x = self.target_x - current_x
        error_y = self.target_y - current_y

        distance = math.sqrt(error_x**2 + error_y**2)

        self.get_logger().info(f"Distance: {distance:.2f} m")

        # Check arrival
        if distance < self.arrival_tolerance:
            self.get_logger().info("Waypoint reached")
            return

        # Normalize direction
        direction_x = error_x / distance
        direction_y = error_y / distance

        # Speed control (m/s)
        speed = 1.5

        # Smooth step toward waypoint
        step_x = current_x + direction_x * speed
        step_y = current_y + direction_y * speed

        # Publish setpoint
        msg = TrajectorySetpoint()
        msg.position = [step_x, step_y, self.target_altitude]
        msg.yaw = math.atan2(direction_y, direction_x)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        self.trajectory_setpoint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    navigator = Navigator()
    rclpy.spin(navigator) # Keep the node running
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
