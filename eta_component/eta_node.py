#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Int32
import math


class EtaPublisher(Node):
    def __init__(self):
        super().__init__('eta_publisher')

        # Subscribers
        self.create_subscription(Path, "/planned_path", self.path_callback, 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(AckermannDrive, "/ackermann_drive_feedback", self.ack_callback, 10)

        # Publisher
        self.eta_pub = self.create_publisher(Int32, "/eta_info", 10)

        # Internal buffers
        self.current_speed = 0.0
        self.path = []

        # Timer (publish ETA at 5 Hz)
        self.create_timer(0.2, self.update_eta)

        self.get_logger().info("🚀 ETA Publisher Running (calculating seconds)…")

    # ============================================================
    # CALLBACKS
    # ============================================================

    def ack_callback(self, msg):
        """Optional: read speed from ackermann controller"""
        self.current_speed = abs(msg.speed)

    def odom_callback(self, msg):
        """Read robot velocity magnitude from odom"""
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_speed = math.sqrt(vx*vx + vy*vy)

    def path_callback(self, msg):
        """Store remaining path coordinates"""
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    # ============================================================
    # ETA CALCULATION
    # ============================================================

    def compute_path_distance(self):
        """Compute remaining path length in meters"""
        if len(self.path) < 2:
            return 0.0

        dist = 0.0
        for i in range(len(self.path) - 1):
            x1, y1 = self.path[i]
            x2, y2 = self.path[i + 1]
            dist += math.dist((x1, y1), (x2, y2))

        return dist

    def update_eta(self):
        """Main ETA calculation"""

        remaining_distance = self.compute_path_distance()

        # If no path → no ETA
        if remaining_distance <= 0.05:
            self.publish_eta(0)
            return

        # If robot nearly stopped → avoid infinite ETA
        if self.current_speed < 0.05:
            eta_seconds = 999   # big number meaning "waiting / not moving"
        else:
            eta_seconds = remaining_distance / self.current_speed

        self.publish_eta(int(eta_seconds))

    def publish_eta(self, eta_sec):
        """Publish ETA in seconds as Int32"""
        msg = Int32()
        msg.data = eta_sec
        self.eta_pub.publish(msg)
        self.get_logger().info(f"⏱ ETA Published: {eta_sec} sec")


def main(args=None):
    rclpy.init(args=args)
    node = EtaPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

