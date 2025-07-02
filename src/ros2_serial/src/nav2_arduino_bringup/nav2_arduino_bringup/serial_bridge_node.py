import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import tf_transformations
import serial
import math

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        self.port = '/dev/ttyACM0'
        self.baud = 57600
        self.ser = serial.Serial(self.port, self.baud, timeout=0.1)

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.br = TransformBroadcaster(self)

        self.ticks_per_meter = 104 / (math.pi * 0.08)
        self.wheel_base = 0.22

        self.last_time = self.get_clock().now()
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_theta = 0.0

        self.create_timer(0.05, self.read_arduino)
        self.get_logger().info("serial bridge started")

    def cmd_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        left = int((linear - angular * self.wheel_base / 2.0) * self.ticks_per_meter / 30.0)
        right = int((linear + angular * self.wheel_base / 2.0) * self.ticks_per_meter / 30.0)
        command = f"m {left} {right}\r"
        self.ser.write(command.encode())

    def read_arduino(self):
        if self.ser.in_waiting:
            line = self.ser.readline().decode(errors='ignore').strip()
            if line.startswith("ODOM:"):
                try:
                    payload = line[5:].strip()
                    x, y, theta = map(float, payload.split(","))
                    self.publish_odom(x, y, theta)
                except Exception as e:
                    self.get_logger().warn(f"ODOM parse error: '{line}' → {e}")

    def publish_odom(self, x, y, theta):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        dx = x - self.last_x
        dy = y - self.last_y
        dtheta = theta - self.last_theta

        vx = dx / dt if dt > 0 else 0.0
        vy = dy / dt if dt > 0 else 0.0
        vtheta = dtheta / dt if dt > 0 else 0.0

        self.last_x = x
        self.last_y = y
        self.last_theta = theta

        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0

        q = tf_transformations.quaternion_from_euler(0, 0, theta)
        t.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.br.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vtheta

        self.odom_pub.publish(odom)
        self.get_logger().info(f"📡 ODOM x={x:.3f}, y={y:.3f}, θ={theta:.2f} rad")

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

