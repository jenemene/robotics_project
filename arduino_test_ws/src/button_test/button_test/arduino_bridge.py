import time

import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import String

PORT = '/dev/ttyACM0'
BAUD = 9600   # must match Serial.begin(...) in the sketch


class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')

        # 1. Open the serial port
        self.ser = serial.Serial(PORT, BAUD, timeout=0.05)
        time.sleep(2)   # the Arduino resets when the port opens; wait for it to boot

        # 2. Publisher: every line the Arduino sends goes out on /arduino/rx
        self.rx_pub = self.create_publisher(String, '/arduino/rx', 10)

        # 3. Subscriber: every message on /arduino/tx is sent to the Arduino
        self.create_subscription(String, '/arduino/tx', self.on_tx, 10)

        # 4. Timer: check the serial port 50 times per second
        self.create_timer(0.02, self.read_serial)
        self.get_logger().info(f'Bridge running on {PORT} at {BAUD} baud')

    def read_serial(self):
        while self.ser.in_waiting > 0:
            line = self.ser.readline().decode(errors='ignore').strip()
            if line:
                self.rx_pub.publish(String(data=line))

    def on_tx(self, msg):
        self.ser.write((msg.data + '\n').encode())


def main():
    rclpy.init()
    node = ArduinoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.ser.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()