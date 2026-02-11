#!/usr/bin/env python3

import math
import csv
import pathlib
from datetime import datetime, timedelta
from collections import deque
from zoneinfo import ZoneInfo

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float64, Int32, Float32


class ETANode(Node):

    def __init__(self, window_size=30):
        super().__init__('eta_node')

        # Subscribers
        self.path_sub = self.create_subscription(Path, '/planned_path', self.path_callback, 100)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 50)
        self.state_sub = self.create_subscription(Int32, '/state', self.state_callback, 50)

        # Publishers
        self.eta_pub = self.create_publisher(Float64, '/eta', 10)
        self.vel_pub = self.create_publisher(Float32, '/vel', 10)

        # Internal state
        self.state = 0
        self.prev_state = 0
        self.mode_calc_eta = 0

        self.path_points = []
        self.path_received = False

        self.departure_time = None          # ROS time
        self.departure_wall_time = None    # Real time

        self.v_min = 0.1
        self.eta_history = deque(maxlen=window_size)
        self.smoothed_eta_sec = None
        self.states = ["IDLE", "DRIVING & PLANNING", "BOARDING", "DROPOFF & DEBOARDING",  "PARKING", "TRIP CANCELLED"]

        self.tz = ZoneInfo("Europe/Berlin")

        self.get_logger().info('✅ ETA node started with smoothing window size: %d' % window_size)

        self.log_dir = pathlib.Path.home() / "eta_logs"
        self.log_dir.mkdir(exist_ok=True)

    def path_callback(self, msg: Path):
        # Updates the planned path points for ETA calculation.
        # Only updates if ETA calculation mode is active.
        # Marks path as received once points are valid.
        if self.mode_calc_eta == 1:
            self.path_points = [
                (pose.pose.position.x, pose.pose.position.y)
                for pose in msg.poses
            ]
            if len(self.path_points) < 2:
                return
            self.path_received = True

    def state_callback(self, msg: Int32):
        # Handles state transitions of the vehicle.
        # Activates/deactivates ETA calculation based on state changes.
        # Initializes CSV logging when starting a trip and closes file when trip ends.
        self.state = msg.data
        if self.prev_state != self.state and self.state in [0,1,2,3,4,5] and self.prev_state in [0,1,2,3,4,5] :
            self.get_logger().info(f"Current state: {self.states[self.state]} | Previous state: {self.states[self.prev_state]}")
            
        if self.state == 1 and self.prev_state == 2: # drving and planning: 1 & boarding: 2
            self.mode_calc_eta = 1
            # Create a filename with timestamp
            timestamp_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            self.csv_path = self.log_dir / f"eta_log_{timestamp_str}.csv"

            self.csv_file = open(self.csv_path, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                "departure_time",
                "eta_hhmmss",
                "smoothed_eta_ros_sec",
                "velocity_mps",
                "current_ros_time_sec"
            ])

            self.get_logger().info(f"ETA node started. Logging to: {self.csv_path}")
        elif self.state == 3 and self.prev_state == 1: #  drop off and deboarding: 3 & drving and planning: 1
            self.mode_calc_eta = 0
            self.smoothed_eta_sec = 0.0
            self.path_received = False
            self.departure_time = None
            self.csv_file.close()
        self.prev_state = self.state

    def odom_callback(self, msg: Odometry):
        # Processes vehicle odometry updates to compute ETA.
        # Calculates current velocity, remaining distance along path, and ETA.
        # Applies smoothing to ETA using a deque window.
        # Publishes smoothed ETA and velocity, and logs them to CSV.
        if self.mode_calc_eta == 1:         
            if not self.path_received:
                return

            # Set departure times once
            if self.departure_time is None:
                self.departure_time = msg.header.stamp.sec + \
                                      msg.header.stamp.nanosec * 1e-9
                self.departure_wall_time = datetime.now(self.tz)

            current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y

            vx = msg.twist.twist.linear.x
            vy = msg.twist.twist.linear.y
            v = math.sqrt(vx * vx + vy * vy)
            v_eff = max(v, self.v_min)

            q = msg.pose.pose.orientation
            yaw = self.yaw_from_quaternion(q)

            remaining_dist, _ = self.compute_remaining_distance(x, y, yaw)

            # Raw ETA in ROS time
            eta_raw = current_time + (remaining_dist / v_eff)

            # Smoothing
            self.eta_history.append(eta_raw)
            self.smoothed_eta_sec = sum(self.eta_history) / len(self.eta_history)

            # Convert ROS ETA → wall clock ETA
            ros_delta = self.smoothed_eta_sec - self.departure_time
            eta_real = self.departure_wall_time + timedelta(seconds=ros_delta)

            eta_str = eta_real.strftime("%H:%M:%S")
            dep_time_str = self.departure_wall_time.strftime("%H:%M:%S")

            # Log
            self.get_logger().info(
                f"Departure: {dep_time_str}, ETA: {eta_str}, Vel: {v:.2f} m/s"
            )

            # CSV write
            self.csv_writer.writerow([
                dep_time_str,                       # Departure wall-clock time HH:MM:SS
                eta_str,                            # ETA wall-clock HH:MM:SS
                self.smoothed_eta_sec,              # Smoothed ETA in ROS seconds since epoch
                f"{v:.3f}",                         # Velocity
                f"{current_time:.3f}"               # Current ROS time in seconds
            ])
            self.csv_file.flush()

            # Publish
            eta_msg = Float64()
            eta_msg.data = self.smoothed_eta_sec
            self.eta_pub.publish(eta_msg)

            vel_msg = Float32()
            vel_msg.data = v
            self.vel_pub.publish(vel_msg)
                

    # --------------------------------------------------
    def yaw_from_quaternion(self, q):
        # Converts a quaternion to a yaw angle (rotation around Z-axis)
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    # --------------------------------------------------
    def compute_remaining_distance(self, x, y, yaw):
        # Computes the remaining distance along the path from the current position.
        # Uses the vehicle heading to ignore points already passed.
        # Returns the total remaining distance and the index of the closest path point ahead.
        hx = math.cos(yaw)
        hy = math.sin(yaw)

        min_idx = None
        min_dist = float('inf')

        for i, (px, py) in enumerate(self.path_points):
            vx = px - x
            vy = py - y
            dot = hx * vx + hy * vy
            if dot <= 0.0:
                continue
            d = math.hypot(vx, vy)
            if d < min_dist:
                min_dist = d
                min_idx = i

        if min_idx is None:
            min_idx = len(self.path_points) - 1

        remaining = math.hypot(
            self.path_points[min_idx][0] - x,
            self.path_points[min_idx][1] - y
        )

        for i in range(min_idx, len(self.path_points) - 1):
            x1, y1 = self.path_points[i]
            x2, y2 = self.path_points[i + 1]
            remaining += math.hypot(x2 - x1, y2 - y1)

        return remaining, min_idx

    # --------------------------------------------------
    def destroy_node(self):
        # Cleans up before shutting down the node
        # CSV file closure handled in state transitions, optional close commented
        # Calls parent destroy_node to release ROS resources
        #self.csv_file.close()
        super().destroy_node()


def main(args=None):
    # ROS2 entry point: initializes ROS, starts node, and spins until shutdown
    rclpy.init(args=args)
    node = ETANode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
