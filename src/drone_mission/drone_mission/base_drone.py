import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, SensorGps, VehicleLocalPosition

class BaseDrone(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers 
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Subscribers
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.gps_sub = self.create_subscription(SensorGps, '/fmu/out/vehicle_gps_position', self.gps_callback, qos_profile)
        self.local_pos_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_pos_callback, qos_profile)

        # Internal State
        self.nav_state = 31 
        self.arm_state = 1
        
        # Telemetry
        self.lat, self.lon, self.alt = 0.0, 0.0, 0.0
        self.local_x, self.local_y, self.local_z = 0.0, 0.0, 0.0

    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state

    def gps_callback(self, msg):
        self.lat = msg.lat * 1e-7
        self.lon = msg.lon * 1e-7
        self.alt = msg.alt * 1e-3

    def local_pos_callback(self, msg):
        self.local_x = msg.x
        self.local_y = msg.y
        self.local_z = msg.z

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)

    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)