import rclpy
import rtde_control
from rclpy.node import Node
from std_msgs.msg import String

# Robot control with ur_rtde (no UR ROS driver, no MoveIt).
# LOCAL mode: the External Control program must be running on the pendant,
# with Host IP = this PC (127.0.0.1 for URSim) and port 50002.
# Do NOT run ur_robot_driver at the same time.

ROBOT_IP = '127.0.0.1'   # URSim. On the real robot: the robot's IP from the pendant

# Target positions, as shown on the pendant (feature = Base): ('movel' or 'movej', [x, y, z, rx, ry, rz])
#   x, y, z    in metres (pendant shows mm)
#   rx, ry, rz rotation vector in radians
#   'movel' = straight line, 'movej' = joints move together (robot does the inverse kinematics)
POSITIONS = [
    ('movel', [0.636, 0.409, 0.008, -3.14, 0.0, 0.0]),                               # position 1
    ('movel', [0.572, 0.500, 0.035, -3.14, 14 * 3.14 / 180, 4 * 3.14 / 180]),        # position 2
    ('movel', [0.640, 0.290, 0.250, -3.14, -14 * 3.14 / 180, 4 * 3.14 / 180]),       # position 3
]
LIN_SPEED = 0.05     # movel: m/s
LIN_ACCEL = 0.3      # movel: m/s^2
JOINT_SPEED = 0.3    # movej: rad/s
JOINT_ACCEL = 0.5    # movej: rad/s^2


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.index = 0       # which position is next

        # connect to the robot through the External Control URCap
        self.get_logger().info(f'Connecting to robot at {ROBOT_IP}... (press Play on the pendant if this hangs)')
        self.robot = rtde_control.RTDEControlInterface(
            ROBOT_IP, 500.0, rtde_control.RTDEControlInterface.FLAG_USE_EXT_UR_CAP, 50002)
        self.get_logger().info('Connected.')

        self.tx_pub = self.create_publisher(String, '/arduino/tx', 10)          # LED on/off
        self.create_subscription(String, '/arduino/rx', self.on_arduino, 10)    # button

        self.get_logger().info('Ready. Press the button to move to position 1.')

    def on_arduino(self, msg):
        if msg.data != 'PRESS':
            return

        kind, pose = POSITIONS[self.index]
        self.tx_pub.publish(String(data='ON'))   # LED on: robot is moving
        self.get_logger().info(f'Moving to position {self.index + 1} ({kind})')

        # these calls WAIT until the robot has arrived, and return True if it went well
        if kind == 'movel':
            ok = self.robot.moveL(pose, LIN_SPEED, LIN_ACCEL)
        else:
            ok = self.robot.moveJ_IK(pose, JOINT_SPEED, JOINT_ACCEL)

        self.tx_pub.publish(String(data='OFF'))  # LED off: arrived (or failed)
        if ok:
            self.get_logger().info('Move finished')
            self.index = (self.index + 1) % len(POSITIONS)
        else:
            self.get_logger().error('Move failed - check the pendant')


def main():
    rclpy.init()
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.robot.stopScript()   # release the robot
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
