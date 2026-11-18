import time

import rclpy
import rtde_control
from rclpy.node import Node
from std_msgs.msg import String

# Pick and place with ur_rtde (no UR ROS driver, no MoveIt).
# LOCAL mode: the External Control program must be running on the pendant,
# with Host IP = this PC (127.0.0.1 for URSim) and port 50002.
# Do NOT run ur_robot_driver at the same time.

ROBOT_IP = '192.168.50.17'   # URSim. On the real robot: the robot's IP from the pendant

# Poses as shown on the pendant (feature = Base): [x, y, z, rx, ry, rz]
#   x, y, z    in metres (pendant shows mm)
#   rx, ry, rz rotation vector in radians
POSES = {
    'pick':  [0.636, 0.409, 0.008, -3.14, 0.0, 0.0],
    'place': [0.572, 0.500, 0.035, -3.14, 14 * 3.14 / 180, 4 * 3.14 / 180],
    'home':  [0.640, 0.290, 0.250, -3.14, -14 * 3.14 / 180, 4 * 3.14 / 180],
}


def above(name, dz=0.20):
    """Same pose as POSES[name], but dz metres higher."""
    p = list(POSES[name])
    p[2] += dz
    return p


# The pick-and-place sequence. Each step is (action, target).
#   'movej' : joints move together, fast, path not straight (robot does the inverse kinematics)
#   'movel' : straight line
#   'grip'  : gripper placeholder (just waits 1 second for now)
SEQUENCE = [
    ('movej', POSES['home']),
    ('movej', above('pick')),     # fast move to above pick
    ('movel', POSES['pick']),     # straight down
    ('grip',  'close'),
    ('movel', above('pick')),     # straight up
    ('movej', above('place')),
    ('movel', POSES['place']),
    ('grip',  'open'),
    ('movel', above('place')),
    ('movej', POSES['home']),
]

LIN_SPEED = 0.05     # movel: m/s
LIN_ACCEL = 0.3      # movel: m/s^2
JOINT_SPEED = 0.3    # movej: rad/s
JOINT_ACCEL = 0.5    # movej: rad/s^2

RUN_ALL = True       # True: one button press = whole sequence. False: one press = one step


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.step = 0   # which step of SEQUENCE is next

        # connect to the robot through the External Control URCap
        self.get_logger().info(f'Connecting to robot at {ROBOT_IP}... (press Play on the pendant if this hangs)')
        flags = (rtde_control.RTDEControlInterface.FLAG_USE_EXT_UR_CAP              # run via External Control (Local mode)
                 | rtde_control.RTDEControlInterface.FLAG_DISABLE_REMOTE_CONTROL_CHECK  # don't ask dashboard (port 29999)
                 | rtde_control.RTDEControlInterface.FLAG_VERBOSE)                     # print what ur_rtde is doing
        self.robot = rtde_control.RTDEControlInterface(ROBOT_IP, 500.0, flags, 50002)
        self.get_logger().info('Connected.')

        self.tx_pub = self.create_publisher(String, '/arduino/tx', 10)          # LED on/off
        self.create_subscription(String, '/arduino/rx', self.on_arduino, 10)    # button

        self.get_logger().info('Ready. Press the button to run pick and place.')

    def on_arduino(self, msg):
        if msg.data != 'PRESS':
            return

        self.tx_pub.publish(String(data='ON'))   # LED on: robot busy

        if RUN_ALL:
            # run every step, starting from wherever we are in the sequence
            while True:
                if not self.run_step():
                    break                        # a step failed: stop
                if self.step == 0:
                    break                        # back at the start: sequence done
        else:
            self.run_step()                      # just one step per press

        self.tx_pub.publish(String(data='OFF'))  # LED off: done (or stopped)

    def run_step(self):
        """Run SEQUENCE[self.step]. Returns True if it went well."""
        action, target = SEQUENCE[self.step]
        self.get_logger().info(f'Step {self.step + 1}/{len(SEQUENCE)}: {action}')

        # moveL/moveJ_IK WAIT until the robot has arrived
        if action == 'movej':
            ok = self.robot.moveJ_IK(target, JOINT_SPEED, JOINT_ACCEL)
        elif action == 'movel':
            ok = self.robot.moveL(target, LIN_SPEED, LIN_ACCEL)
        elif action == 'grip':
            # Later: send 'CLOSE'/'OPEN' to the Arduino and wait for it to answer.
            self.get_logger().info(f'(placeholder) gripper {target}')
            time.sleep(1.0)
            ok = True

        if not ok:
            self.get_logger().error(f'Step {self.step + 1} failed. Next press retries it.')
            return False

        self.step = (self.step + 1) % len(SEQUENCE)   # go to the next step
        if self.step == 0:
            self.get_logger().info('Pick and place finished')
        return True


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
