import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String

# Target poses in the robot base frame: [x, y, z, rx, ry, rz]. I think the urrobot driver can handle inverse kinematics itself, which is very nice
#   x, y, z    position in metres
#   rx, ry, rz orientation as a rotation vector, in radians
POSITIONS = [
    [0.40, -0.20, 0.50, 0.0, 3.14, 0.0],   # position 1
    [0.40, 0.20, 0.50, 0.0, 3.14, 0.0],    # position 2
    [0.40, 0.20, 0.30, 0.0, 3.14, 0.0],    # position 3
]
ACCEL = 0.3       # m/s^2
SPEED = 0.05      # m/s
TOLERANCE = 0.002   # metres, how close counts as "arrived" - we check this to turn the LED off when its arrived


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.index = 0       # which position is next
        self.target = None   # (x, y, z) of the move in progress, or None

        #publishers
        self.script_pub = self.create_publisher(
            String, '/urscript_interface/script_command', 10) #publisher that publishes to the ur_driver topic created by urdriver itself
        
        self.tx_pub = self.create_publisher(String, '/arduino/tx', 10) #publisher that publishes to the arduino - for now this mainly just publishes whether the led should be on or off

        #subscribers
        self.create_subscription(String, '/arduino/rx', self.on_arduino, 10) #subscriber to the topic that the arduino_bridge publishes to. right now for listening to button press
        #should be read like "message on the topic of arduino/rx should go to on_arduino"
        #(message can be more than one thing - im pretty sure msg.data is what were actually looking for)
        self.create_subscription(
            PoseStamped, '/tcp_pose_broadcaster/pose', self.on_pose, 10) #subscriber to topic that tells the pose of the TCP.

        #prints in the terminal. Convenient.
        self.get_logger().info('Ready. Press the button to move to position 1.')

    def on_arduino(self, msg):
        if msg.data != 'PRESS':
            return

        pose = POSITIONS[self.index] #gets the current position
        self.script_pub.publish(
            String(data=f'movel(p{pose}, a={ACCEL}, v={SPEED})\n')) #publishes to ur topic that it should do movel to the pose with the specified vel and acc
        self.tx_pub.publish(String(data='ON'))   # LED on: robot is moving - publishes to arduino/tx node
        self.target = pose[:3]                   # only x, y, z are checked
        self.get_logger().info(f'Moving to position {self.index + 1}')

        self.index = (self.index + 1) % len(POSITIONS)

    def on_pose(self, msg):
        if self.target is None:
            return   # no move in progress
        p = msg.pose.position #extraxts position from the msg from the ur pose topic
        distance = math.dist((p.x, p.y, p.z), self.target) #calculates how close it is BASED ONLY ON POSITION - THIS IS FOR TESTING PURPOSES (also all orientations are the same)

        #if statement that publishes to arduino/tx that it should turn the led off its on the position
        if distance < TOLERANCE:
            self.target = None
            self.tx_pub.publish(String(data='OFF'))   # LED off: arrived
            self.get_logger().info('Move finished')


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