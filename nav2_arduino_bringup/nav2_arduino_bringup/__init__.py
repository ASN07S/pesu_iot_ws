import rclpy
from rclpy.node import Node
import serial
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import time

class SerialBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 9600)
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value

        self.ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2)  # wait for Arduino reset

        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.br = TransformBroadcaster(self)

        self.timer = self.create_timer(0.05, self.read_serial)

    def cmd_callback(self, msg):
        cmd = f"CMD:{msg.linear.x:.3f},{msg.angular.z:.3f}\n"
        self.ser.write(cmd.encode())

    def read_serial(self):
        line = self.ser.readline().decode().strip()
        if line.startswith("ODOM:"):
            try:
                data = line[5:].split(',')
                x, y, theta = float(data[0]), float(data[1]), float(data[2])
                now = self.get_clock().now().to_msg()

                odom = Odometry()
                odom.header.stamp = now
                odom.header.frame_id = 'odom'
                odom.child_frame_id = 'base_link'
                odom.pose.pose.position.x = x
                odom.pose.pose.position.y = y
                odom.pose.pose.orientation.z = math.sin(theta/2)
                odom.pose.pose.orientation.w = math.cos(theta/2)
                self.odom_pub.publish(odom)

                t = TransformStamped()
                t.header.stamp = now
                t.header.frame_id = 'odom'
                t.child_frame_id = 'base_link'
                t.transform.translation.x = x
                t.transform.translation.y = y
                t.transform.translation.z = 0.0
                t.transform.rotation.z = math.sin(theta / 2)
                t.transform.rotation.w = math.cos(theta / 2)
                self.br.sendTransform(t)

            except Exception as e:
                self.get_logger().warn(f"Parse error: {e}")

def main():
    rclpy.init()
    node = SerialBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

