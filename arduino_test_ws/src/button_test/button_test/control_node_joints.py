import math

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint

# Works in LOCAL mode: the External Control program must be running on the pendant.
# Motion goes through the driver's trajectory controller instead of raw URScript.

JOINT_NAMES = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]

# Joint angles in DEGREES, copied from the pendant's Move tab (Base, Shoulder, Elbow, Wrist 1, 2, 3).
# Fill in by jogging the robot to each target and reading the joint values.
POSITIONS_DEG = [
    # [base, shoulder, elbow, wrist1, wrist2, wrist3],   # position 1
]
MOVE_TIME = 8   # seconds per move - larger = slower

ACTION = '/scaled_joint_trajectory_controller/follow_joint_trajectory'


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.index = 0       # which position is next
        self.busy = False    # True while a move is running

        # action client: sends a trajectory and later gets told when it is done
        self.client = ActionClient(self, FollowJointTrajectory, ACTION)

        self.tx_pub = self.create_publisher(String, '/arduino/tx', 10)
        self.create_subscription(String, '/arduino/rx', self.on_arduino, 10)
        self.get_logger().info('Ready. Press the button to move to position 1.')

    def on_arduino(self, msg):
        if msg.data != 'PRESS' or self.busy:
            return
        if not POSITIONS_DEG:
            self.get_logger().error('POSITIONS_DEG is empty - fill it in first')
            return
        if not self.client.server_is_ready():
            self.get_logger().error('Controller not ready - is External Control running on the pendant?')
            return

        # one trajectory point: the target joint angles, reached after MOVE_TIME seconds
        point = JointTrajectoryPoint()
        point.positions = [math.radians(a) for a in POSITIONS_DEG[self.index]]
        point.time_from_start = Duration(sec=MOVE_TIME)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES
        goal.trajectory.points = [point]

        self.busy = True
        self.tx_pub.publish(String(data='ON'))   # LED on: robot is moving
        self.get_logger().info(f'Moving to position {self.index + 1}')
        self.client.send_goal_async(goal).add_done_callback(self.on_goal_response)
        self.index = (self.index + 1) % len(POSITIONS_DEG)

    def on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal rejected by the controller')
            self.finish()
            return
        handle.get_result_async().add_done_callback(self.on_result)

    def on_result(self, future):
        code = future.result().result.error_code   # 0 means success
        if code == 0:
            self.get_logger().info('Move finished')
        else:
            self.get_logger().error(f'Move failed, error code {code}')
        self.finish()

    def finish(self):
        self.busy = False
        self.tx_pub.publish(String(data='OFF'))   # LED off


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
