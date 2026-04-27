import rclpy
import math
from .base_drone import BaseDrone 
from px4_msgs.msg import TrajectorySetpoint

class Navigator(BaseDrone):
    def __init__(self):
        super().__init__('navigator_node')
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # 10m Square Path [X, Y, Z]
        self.waypoints = [
            [0.0, 0.0, -5.0],   # Takeoff
            [10.0, 0.0, -5.0],  # Waypoint 1
            [10.0, 10.0, -5.0], # Waypoint 2
            [0.0, 10.0, -5.0],  # Waypoint 3
            [0.0, 0.0, -5.0]    # Back to start
        ]
        self.current_wp_idx = 0
        
        self.mission_state = "WARMUP"
        self.counter = 0

    def timer_callback(self):
        self.publish_offboard_control_mode()

        # Regular stream (Every 2 seconds)
        if self.counter % 20 == 0:
            self.get_logger().info(f"Flying... Current State: {self.mission_state}")

        if self.mission_state == "WARMUP":
            self.publish_trajectory_setpoint(0.0, 0.0, 0.0)
            if self.counter > 20: 
                self.get_logger().info("Mission Start: Arming...")
                self.engage_offboard_mode()
                self.arm()
                self.mission_state = "WAYPOINT_NAV"
            self.counter += 1

        elif self.mission_state == "WAYPOINT_NAV":
            target = self.waypoints[self.current_wp_idx]
            tx, ty, tz = target[0], target[1], target[2]

            self.publish_trajectory_setpoint(tx, ty, tz)

            dist = math.sqrt((tx - self.local_x)**2 + (ty - self.local_y)**2 + (tz - self.local_z)**2)

            if dist < 0.6:
                # RECENT ARRIVAL LOG
                self.get_logger().info(f"--- ARRIVED AT WAYPOINT {self.current_wp_idx} ---")
                self.get_logger().info(f"SNAPSHOT LOCAL: X={self.local_x:.3f}, Y={self.local_y:.3f}, Z={self.local_z:.3f}")
                self.get_logger().info(f"SNAPSHOT GPS  : Lat={self.lat:.7f}, Lon={self.lon:.7f}, Alt={self.alt:.2f}")
                
                if self.current_wp_idx < len(self.waypoints) - 1:
                    self.current_wp_idx += 1
                else:
                    self.get_logger().info("All waypoints confirmed. Landing.")
                    self.land()
                    self.mission_state = "LANDING"
            
            self.counter += 1

        elif self.mission_state == "LANDING":
            self.counter += 1

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
        navigator.get_logger().info("User interrupted.")
    finally:
        if rclpy.ok():
            navigator.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()