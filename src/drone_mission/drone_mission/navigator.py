import rclpy
from .base_drone import BaseDrone 
from px4_msgs.msg import TrajectorySetpoint

class Navigator(BaseDrone):
    def __init__(self):
        super().__init__('navigator_node')
        
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.target_altitude = -5.0 
        self.counter = 0

    def timer_callback(self):

        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()


        if self.counter < 20:
            if self.counter == 0:
                self.get_logger().info("Warm up")
            self.counter += 1
            return


        if self.counter < 70:
            if self.counter % 10 == 0:
                self.get_logger().info(f"Takeoff (Attempt {self.counter//10})...")
            
            self.arm()
            self.engage_offboard_mode()
            self.counter += 1
        
        else:
            if self.counter == 70:
                self.get_logger().info("Hover")
            self.counter += 1

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = [0.0, 0.0, self.target_altitude]
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