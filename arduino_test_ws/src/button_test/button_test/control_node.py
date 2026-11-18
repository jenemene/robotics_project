import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String

# Pick-and-place sequence with plain URScript (no MoveIt).
# Needs the pendant in REMOTE mode. No External Control program needed.
# The robot does the inverse kinematics itself.

# Poses as shown on the pendant (feature = Base): [x, y, z, rx, ry, rz]
#   x, y, z    in metres (pendant shows mm)
#   rx, ry, rz rotation vector in radians
# The poses are for the TCP that is active on the pendant.
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


# The sequence: (move type, target). Edit this list to change what the robot does.
#   'movej' : joints move together, fast, path is not a straight line
#   'movel' : straight line
#   'grip'  : gripper placeholder (just waits GRIP_TIME for now)
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

RUN_ALL = False      # True: one button press = whole sequence. False: one press = one step

LIN_ACCEL = 0.3     # movel: m/s^2
LIN_SPEED = 0.05    # movel: m/s
JOINT_ACCEL = 0.5   # movej: rad/s^2 (movej speeds are JOINT speeds, not tool speeds)
JOINT_SPEED = 0.3   # movej: rad/s
TOLERANCE = 0.002   # metres, how close counts as "arrived"
GRIP_TIME = 1.0     # seconds the placeholder gripper "takes"


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.step = 0          # which step of SEQUENCE is next
        self.target = None     # (x, y, z) of the move in progress, or None

        # publishers
        self.script_pub = self.create_publisher(
            String, '/urscript_interface/script_command', 10)   # URScript to the robot
        self.tx_pub = self.create_publisher(String, '/arduino/tx', 10)   # LED on/off

        # subscribers
        self.create_subscription(String, '/arduino/rx', self.on_arduino, 10)   # button
        self.create_subscription(
            PoseStamped, '/tcp_pose_broadcaster/pose', self.on_pose, 10)   # where the TCP is

        self.get_logger().info(f'Ready. {len(SEQUENCE)} steps. Press the button to start.')

    def on_arduino(self, msg):
        if msg.data != 'PRESS' or self.busy():
            return
        self.tx_pub.publish(String(data='ON'))   # LED on: busy
        self.run_step()

    def busy(self):
        return self.target is not None or getattr(self, 'grip_timer', None) is not None

    def run_step(self):
        kind, target = SEQUENCE[self.step]
        self.get_logger().info(f'Step {self.step + 1}/{len(SEQUENCE)}: {kind} {target}')

        if kind == 'movej':
            self.script_pub.publish(String(
                data=f'movej(p{target}, a={JOINT_ACCEL}, v={JOINT_SPEED})\n'))
            self.target = target[:3]   # only x, y, z are checked
        elif kind == 'movel':
            self.script_pub.publish(String(
                data=f'movel(p{target}, a={LIN_ACCEL}, v={LIN_SPEED})\n'))
            self.target = target[:3]
        elif kind == 'grip':
            # Later: send 'CLOSE'/'OPEN' to the Arduino and wait for it to answer.
            self.get_logger().info(f'(placeholder) gripper {target}')
            self.grip_timer = self.create_timer(GRIP_TIME, self.on_grip_done)

    def on_pose(self, msg):
        if self.target is None:
            return   # no move in progress
        p = msg.pose.position
        distance = math.dist((p.x, p.y, p.z), self.target)
        if distance < TOLERANCE:
            self.target = None
            self.step_done()

    def on_grip_done(self):
        self.grip_timer.cancel()   # only fire once
        self.grip_timer = None
        self.step_done()

    def step_done(self):
        self.step = (self.step + 1) % len(SEQUENCE)
        if self.step == 0:
            self.get_logger().info('Sequence finished')
            self.tx_pub.publish(String(data='OFF'))
        elif RUN_ALL:
            self.run_step()   # go straight on to the next step
        else:
            self.tx_pub.publish(String(data='OFF'))   # wait for the next press


def main():
    rclpy.init()
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
