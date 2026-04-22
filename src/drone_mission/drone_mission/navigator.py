import rclpy
from .base_drone import BaseDrone 
from px4_msgs.msg import TrajectorySetpoint

class Navigator(BaseDrone):
    def __init__(self):
        super().__init__('navigator_node')
        

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.takeoff_alt = -5.0 
        self.waypoint_x = 5.0
        self.waypoint_y = 5.0
        
        self.counter = 0
        self.mission_state = "WARMUP"

    def timer_callback(self):

        self.publish_offboard_control_mode()

        if self.mission_state == "WARMUP":

            self.publish_trajectory_setpoint(0.0, 0.0, 0.0)
            
            if self.counter > 20: 
                self.get_logger().info("Requesting Offboard and Arming...")
                self.engage_offboard_mode()
                self.arm()
                self.mission_state = "TAKEOFF"
            self.counter += 1

        elif self.mission_state == "TAKEOFF":

            self.publish_trajectory_setpoint(0.0, 0.0, self.takeoff_alt)
            

            if self.counter > 100: 
                self.get_logger().info(f"Target altitude reached. Moving to X:{self.waypoint_x} Y:{self.waypoint_y}")
                self.mission_state = "MOVE"
            self.counter += 1

        elif self.mission_state == "MOVE":

            self.publish_trajectory_setpoint(self.waypoint_x, self.waypoint_y, self.takeoff_alt)

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
        navigator.get_logger().info("User interrupted. Shutting down.")
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()