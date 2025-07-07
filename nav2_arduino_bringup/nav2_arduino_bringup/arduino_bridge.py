import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
import serial
import math

class ArduinoOdometryNode(Node):
    def __init__(self):
        super().__init__('arduino_odometry_node')

        # Set your Arduino serial port here
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.br = TransformBroadcaster(self)

        self.timer = self.create_timer(0.05, self.read_serial)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def read_serial(self):
        try:
            line = self.ser.readline().decode().strip()
            if line.startswith("ODOM:"):
                data = line[5:].split(',')
                self.x = float(data[0])
                self.y = float(data[1])
                self.theta = float(data[2])

                self.publish_odom()
                self.send_tf()

        except Exception as e:
            self.get_logger().warn(f"Serial read error: {e}")

    def publish_odom(self):
        msg = Odometry()
        now = self.get_clock().now().to_msg()

        msg.header.stamp = now
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0

        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        self.publisher_.publish(msg)

    def send_tf(self):
        t = TransformStamped()
        now = self.get_clock().now().to_msg()

        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.br.sendTransform(t)

    def cmd_vel_callback(self, msg: Twist):
        # Extract linear and angular velocities
        vx = msg.linear.x
        omega = msg.angular.z

        # Format for Arduino: CMD:<vx>,<omega>
        cmd_str = f"CMD:{vx:.3f},{omega:.3f}\n"
        try:
            self.ser.write(cmd_str.encode())
        except Exception as e:
            self.get_logger().warn(f"Serial write error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
