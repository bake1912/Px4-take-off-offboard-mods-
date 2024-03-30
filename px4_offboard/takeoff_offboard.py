from time import sleep
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus

class OffboardControl(Node):
    def __init__(self) -> None:
        super().__init__('offboard_control')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.offboard_setpoint_counter = 0
        self.timer = self.create_timer(0.1, self.heartbeat_timer_callback)
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.is_armed = False
        self.is_health_ok = False
        self.subscription = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.listener_callback,
            qos_profile)

    def listener_callback(self, msg):
        self.is_armed = msg.arming_state == 2
        self.is_health_ok = msg.pre_flight_checks_pass

    def publish_offboard_control_heartbeat_signal(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params) -> None:
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_publisher.publish(msg)

    def arm_drone(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def takeoff(self):
        print('Taking off...')
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1=0.0, param2=0.0, param3=0.0, param4=0.0)

    def prepare_for_offboard(self):
        if not self.is_armed:
            self.arm_drone()
        self.takeoff()

    def engage_offboard_mode(self):
        self.prepare_for_offboard()
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def heartbeat_timer_callback(self) -> None:
        if not self.is_health_ok:
            self.get_logger().info("Pre-flight checks...")
            sleep(1)
            return
        self.publish_offboard_control_heartbeat_signal()
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.get_logger().info("Switched to offboard mode")
        if self.offboard_setpoint_counter < 11:
            self.get_logger().info("Waiting 1 second before switch to offboard mode.")
            self.offboard_setpoint_counter += 1

def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)