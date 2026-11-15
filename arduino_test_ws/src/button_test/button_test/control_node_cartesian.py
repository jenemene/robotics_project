import math

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

# Works in LOCAL mode, like cartesian_square.py:
#   - External Control program running on the pendant
#   - MoveIt running: ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur12e launch_rviz:=false
# Moves in a straight line (like movel) to poses copied from the pendant.

# Target poses as shown on the pendant, feature = Base: [x, y, z, rx, ry, rz]
#   x, y, z    in METRES (pendant shows mm)
#   rx, ry, rz rotation vector in radians
# Read them with the pendant TCP set to zero (flange), because link 'tool0' below is the flange.
POSITIONS = [
    [0.636, 0.409, 0.008, -3.14, 0.0, 0.0], 
    [0.572, 0.500, 0.035, -3.14, 14*3.14/180, 4*3.14/180],
    [0.640, 0.290, 0.250, -3.14, -14*3.14/180, 4*3.14/180],  # position 1 - CHECK z before running
]
SPEED_SCALE = 0.1   # fraction of max joint speed/acceleration

FRAME = 'base'   # UR's 'base' frame = pendant Base. NOT 'base_link' (rotated 180 deg about z)
LINK = 'tool0'   # flange


def pose_from_ur(p):
    """Convert [x, y, z, rx, ry, rz] (UR rotation vector) to a ROS Pose (quaternion)."""
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = p[0], p[1], p[2]
    rx, ry, rz = p[3], p[4], p[5]
    angle = math.sqrt(rx * rx + ry * ry + rz * rz)
    if angle < 1e-9:
        pose.orientation.w = 1.0
        return pose
    s = math.sin(angle / 2) / angle   # axis * sin(angle/2), axis = r / angle
    pose.orientation.x = rx * s
    pose.orientation.y = ry * s
    pose.orientation.z = rz * s
    pose.orientation.w = math.cos(angle / 2)
    return pose


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.index = 0
        self.busy = False

        self.plan_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.exec_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')

        self.tx_pub = self.create_publisher(String, '/arduino/tx', 10)
        self.create_subscription(String, '/arduino/rx', self.on_arduino, 10)
        self.get_logger().info('Ready. Press the button to move to position 1.')

    def on_arduino(self, msg):
        if msg.data != 'PRESS' or self.busy:
            return
        if not self.plan_client.service_is_ready():
            self.get_logger().error('MoveIt not running? /compute_cartesian_path not available')
            return

        # 1. Ask MoveIt for a straight-line path from where the robot is now to the target
        req = GetCartesianPath.Request()
        req.header.frame_id = FRAME
        req.group_name = 'ur_manipulator'
        req.link_name = LINK
        req.waypoints = [pose_from_ur(POSITIONS[self.index])]
        req.max_step = 0.005          # interpolate every 5 mm
        req.avoid_collisions = True
        req.max_velocity_scaling_factor = SPEED_SCALE
        req.max_acceleration_scaling_factor = SPEED_SCALE

        self.busy = True
        self.tx_pub.publish(String(data='ON'))   # LED on
        self.get_logger().info(f'Planning to position {self.index + 1}')
        self.plan_client.call_async(req).add_done_callback(self.on_plan)

    def on_plan(self, future):
        res = future.result()
        if res.fraction < 0.99:
            self.get_logger().error(f'Only {res.fraction * 100:.0f}% of the path is possible - not moving')
            self.finish()
            return
        if not self.exec_client.server_is_ready():
            self.get_logger().error('/execute_trajectory not available')
            self.finish()
            return

        # 2. Execute the planned trajectory on the robot
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = res.solution
        self.exec_client.send_goal_async(goal).add_done_callback(self.on_goal_response)
        self.index = (self.index + 1) % len(POSITIONS)

    def on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Trajectory rejected')
            self.finish()
            return
        handle.get_result_async().add_done_callback(self.on_result)

    def on_result(self, future):
        code = future.result().result.error_code.val   # 1 = SUCCESS in MoveIt
        if code == 1:
            self.get_logger().info('Move finished')
        else:
            self.get_logger().error(f'Move failed, MoveIt error code {code}')
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
