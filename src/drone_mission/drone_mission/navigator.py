import rclpy
import math
from .base_drone import BaseDrone 
from px4_msgs.msg import TrajectorySetpoint
from sensor_msgs.msg import NavSatFix


class Navigator(BaseDrone):
    def __init__(self):
        super().__init__('navigator_node')
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.mission_state = "WARMUP"
        self.counter = 0
        self.home_lat = None
        self.home_lon = None


        self.target_gps_lat = 47.3978  
        self.target_gps_lon = 8.5457
        
        self.waypoints = [
            [0.0, 0.0, -5.0],    # Waypoint 0: Takeoff
            [10.0, 0.0, -5.0],   # Waypoint 1: 10m East
            [10.0, 10.0, -5.0]   # Waypoint 2: 10m East, 10m South
        ]
        self.current_wp_idx = 0

    def gps_to_ned(self, lat, lon):

        R = 6378137.0 
        d_lat = math.radians(lat - self.home_lat)
        d_lon = math.radians(lon - self.home_lon)
        x = R * d_lon * math.cos(math.radians(self.home_lat))
        y = R * d_lat
        return x, y

    def timer_callback(self):
        self.publish_offboard_control_mode()


        if self.home_lat is None and self.lat != 0.0:
            self.home_lat = self.lat
            self.home_lon = self.lon
            self.get_logger().info(f"Home set: {self.home_lat}, {self.home_lon}")
            

            gx, gy = self.gps_to_ned(self.target_gps_lat, self.target_gps_lon)
            self.get_logger().info(f"GPS Target converted to Local: X={gx:.2f}, Y={gy:.2f}")
            


        if self.mission_state == "WARMUP":
            self.publish_trajectory_setpoint(0.0, 0.0, 0.0)
            if self.counter > 20 and self.home_lat is not None: 
                self.engage_offboard_mode()
                self.arm()
                self.mission_state = "WAYPOINT_NAV"
            self.counter += 1

        elif self.mission_state == "WAYPOINT_NAV":
            target = self.waypoints[self.current_wp_idx]
            tx, ty, tz = target[0], target[1], target[2]
            self.publish_trajectory_setpoint(tx, ty, tz)

            dist = math.sqrt((tx - self.local_x)**2 + (ty - self.local_y)**2 + (tz - self.local_z)**2)
            if dist < 0.8:
                self.get_logger().info(f"Reached Waypoint {self.current_wp_idx}")
                if self.current_wp_idx < len(self.waypoints) - 1:
                    self.current_wp_idx += 1
                else:
                    self.get_logger().info("Mission Complete. Initiating Land.")
                    self.land()
                    self.mission_state = "LANDING"

    def publish_trajectory_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.0 
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        self.trajectory_setpoint_pub.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    navigator = Navigator()
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()