import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import hypot, atan2, pi
import time

class GoalStopper(Node):
    def __init__(self):
        super().__init__('goal_stop_controller')

        # Goal to monitor
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_theta = 0.0
        self.goal_set = False

        # Thresholds
        self.dist_thresh = 0.05  # meters
        self.angle_thresh = 0.1  # radians

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Set goal once when this script starts (you can modify this dynamically)
        self.goal_x = 1.0  # change as per your 2D goal
        self.goal_y = 0.0
        self.goal_theta = 0.0
        self.goal_set = True

    def odom_callback(self, msg):
        if not self.goal_set:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Orientation → yaw
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        theta = 2 * atan2(z, w)

        dist = hypot(self.goal_x - x, self.goal_y - y)
        angle_diff = abs(self.goal_theta - theta)

        if dist < self.dist_thresh and angle_diff < self.angle_thresh:
            self.get_logger().info("Goal reached! Stopping robot.")
            stop = Twist()
            self.cmd_pub.publish(stop)
            self.goal_set = False  # Optional: only stop once

def main(args=None):
    rclpy.init(args=args)
    node = GoalStopper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

