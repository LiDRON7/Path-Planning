import rclpy
from .base_drone import BaseDrone 
from px4_msgs.msg import TrajectorySetpoint

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

    def timer_callback(self):
        """runs every 0.1 seconds"""
        
        # Always send the Offboard Heartbeat
        self.publish_offboard_control_mode()

        # Send the  position 
        self.publish_trajectory_setpoint()

        # Attempt to Arm and Take Command (Once)
        if not self.mission_started:
            self.arm()
            self.engage_offboard_mode()
            self.mission_started = True

    def publish_trajectory_setpoint(self):
        """Send drone"""
        msg = TrajectorySetpoint()
        msg.position = [2.0, 2.0, self.target_altitude] # [X, Y, Z]
        msg.yaw = 0.0 # Face North
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
