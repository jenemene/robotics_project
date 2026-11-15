import math

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String

# Pick-and-place SEQUENCE test. Works in LOCAL mode:
#   - External Control program running on the pendant
#   - MoveIt running: ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur12e launch_rviz:=false
#
# Move types (like UR commands):
#   'PTP'  ~ movej : Pilz point-to-point, joints move together, fast, path not straight
#   'LIN'  ~ movel : Pilz straight line
#   'CART' ~ movel : straight line via compute_cartesian_path (the method tested earlier)
#   'GRIP'         : gripper placeholder (just waits GRIP_TIME for now)

# ---------------------------------------------------------------------------
# 1. NAMED POSES, as shown on the pendant (feature = Base, TCP = flange):
#    [x, y, z, rx, ry, rz]  metres + rotation vector in radians
# ---------------------------------------------------------------------------
POSES = {
    'pick':  [0.636, 0.409, 0.008, -3.14, 0.0, 0.0],
    'place': [0.572, 0.500, 0.035, -3.14, 14 * 3.14 / 180, 4 * 3.14 / 180],
    'home':  [0.640, 0.290, 0.250, -3.14, -14 * 3.14 / 180, 4 * 3.14 / 180],
}


def above(name, dz=0.10):
    """Same pose as POSES[name], but dz metres higher."""
    p = list(POSES[name])
    p[2] += dz
    return p


# ---------------------------------------------------------------------------
# 2. THE SEQUENCE: edit this list to change what the robot does.
#    Each step is (action, target). Change 'PTP' <-> 'LIN' to change the move type.
# ---------------------------------------------------------------------------
SEQUENCE = [
    ('PTP',  POSES['home']),
    ('PTP',  above('pick',dz=0.5)),      # fast move to 10 cm above pick
    ('LIN',  POSES['pick']),      # straight down
    ('GRIP', 'close'),
    ('LIN',  above('pick')),      # straight up
    ('PTP',  above('place')),
    ('LIN',  POSES['place']),
    ('GRIP', 'open'),
    ('LIN',  above('place')),
    ('PTP',  POSES['home']),
]

RUN_ALL = False     # False: one button press = one step (good for testing)
                    # True:  one button press = the whole sequence
SPEED_SCALE = 0.1   # fraction of max speed/acceleration
GRIP_TIME = 1.0     # seconds the placeholder gripper "takes"

FRAME = 'base'      # UR's 'base' frame = pendant Base (NOT 'base_link')
LINK = 'tool0'      # flange
GROUP = 'ur_manipulator'
PILZ = 'pilz_industrial_motion_planner'


def pose_from_ur(p):
    """Convert [x, y, z, rx, ry, rz] (UR rotation vector) to a ROS Pose (quaternion)."""
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = p[0], p[1], p[2]
    rx, ry, rz = p[3], p[4], p[5]
    angle = math.sqrt(rx * rx + ry * ry + rz * rz)
    if angle < 1e-9:
        pose.orientation.w = 1.0
        return pose
    s = math.sin(angle / 2) / angle
    pose.orientation.x = rx * s
    pose.orientation.y = ry * s
    pose.orientation.z = rz * s
    pose.orientation.w = math.cos(angle / 2)
    return pose


def pose_goal(pose):
    """Wrap a Pose as a MoveIt goal: 'tool0 must end up here' (1 mm / 0.01 rad tolerance)."""
    pc = PositionConstraint()
    pc.header.frame_id = FRAME
    pc.link_name = LINK
    pc.constraint_region.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.001])]
    pc.constraint_region.primitive_poses = [pose]
    pc.weight = 1.0

    oc = OrientationConstraint()
    oc.header.frame_id = FRAME
    oc.link_name = LINK
    oc.orientation = pose.orientation
    oc.absolute_x_axis_tolerance = 0.01
    oc.absolute_y_axis_tolerance = 0.01
    oc.absolute_z_axis_tolerance = 0.01
    oc.weight = 1.0

    c = Constraints()
    c.position_constraints = [pc]
    c.orientation_constraints = [oc]
    return c


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.step = 0          # index into SEQUENCE
        self.busy = False

        self.move_client = ActionClient(self, MoveGroup, '/move_action')                      # PTP / LIN
        self.cart_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')    # CART
        self.exec_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')       # CART

        self.tx_pub = self.create_publisher(String, '/arduino/tx', 10)
        self.create_subscription(String, '/arduino/rx', self.on_arduino, 10)
        self.get_logger().info(f'Ready. {len(SEQUENCE)} steps. Press the button to start.')

    # ----------------------------------------------------------- button
    def on_arduino(self, msg):
        if msg.data != 'PRESS' or self.busy:
            return
        self.busy = True
        self.tx_pub.publish(String(data='ON'))   # LED on while busy
        self.run_step()

    # ----------------------------------------------------------- sequence logic
    def run_step(self):
        kind, target = SEQUENCE[self.step]
        self.get_logger().info(f'Step {self.step + 1}/{len(SEQUENCE)}: {kind} {target}')
        if kind in ('PTP', 'LIN'):
            self.move_pilz(kind, target)
        elif kind == 'CART':
            self.move_cart(target)
        elif kind == 'GRIP':
            self.gripper(target)
        else:
            self.get_logger().error(f'Unknown step type {kind}')
            self.step_done(False)

    def step_done(self, success):
        if not success:
            self.get_logger().error(f'Step {self.step + 1} failed - stopping. Next press retries it.')
            self.finish()
            return
        self.step = (self.step + 1) % len(SEQUENCE)
        if self.step == 0:
            self.get_logger().info('Sequence finished')
            self.finish()
        elif RUN_ALL:
            self.run_step()            # chain straight into the next step
        else:
            self.finish()              # wait for the next button press

    def finish(self):
        self.busy = False
        self.tx_pub.publish(String(data='OFF'))   # LED off

    # ----------------------------------------------------------- PTP / LIN (Pilz)
    def move_pilz(self, kind, target):
        if not self.move_client.server_is_ready():
            self.get_logger().error('/move_action not available - is MoveIt running?')
            self.step_done(False)
            return
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = GROUP
        req.pipeline_id = PILZ
        req.planner_id = kind                       # 'PTP' or 'LIN'
        req.goal_constraints = [pose_goal(pose_from_ur(target))]
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = SPEED_SCALE
        req.max_acceleration_scaling_factor = SPEED_SCALE
        goal.planning_options.plan_only = False     # plan AND execute
        self.move_client.send_goal_async(goal).add_done_callback(self.on_goal_response)

    # ----------------------------------------------------------- CART (tested method)
    def move_cart(self, target):
        if not self.cart_client.service_is_ready():
            self.get_logger().error('/compute_cartesian_path not available')
            self.step_done(False)
            return
        req = GetCartesianPath.Request()
        req.header.frame_id = FRAME
        req.group_name = GROUP
        req.link_name = LINK
        req.waypoints = [pose_from_ur(target)]
        req.max_step = 0.005
        req.avoid_collisions = True
        req.max_velocity_scaling_factor = SPEED_SCALE
        req.max_acceleration_scaling_factor = SPEED_SCALE
        self.cart_client.call_async(req).add_done_callback(self.on_cart_plan)

    def on_cart_plan(self, future):
        res = future.result()
        if res.fraction < 0.99:
            self.get_logger().error(f'Only {res.fraction * 100:.0f}% of the line is possible')
            self.step_done(False)
            return
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = res.solution
        self.exec_client.send_goal_async(goal).add_done_callback(self.on_goal_response)

    # ----------------------------------------------------------- shared result handling
    def on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal rejected')
            self.step_done(False)
            return
        handle.get_result_async().add_done_callback(self.on_result)

    def on_result(self, future):
        code = future.result().result.error_code.val   # 1 = SUCCESS in MoveIt
        if code != 1:
            self.get_logger().error(f'MoveIt error code {code}')
        self.step_done(code == 1)

    # ----------------------------------------------------------- gripper placeholder
    def gripper(self, command):
        # Later: publish 'CLOSE'/'OPEN' on /arduino/tx and wait for the Arduino to report success.
        self.get_logger().info(f'(placeholder) gripper {command}')
        self.grip_timer = self.create_timer(GRIP_TIME, self.on_grip_done)

    def on_grip_done(self):
        self.grip_timer.cancel()   # one-shot
        self.step_done(True)


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
