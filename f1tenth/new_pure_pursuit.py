"""
Python Pure Pursuit node for path following
"""

import rclpy
import csv
import math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped


class PurePursuit(Node):
    def __init__(self):
        super().__init__("pure_pursuit")
        self.waypoints = self.load_waypoints(
            "/home/nvidia/team2TEMP/src/f1tenth/VipPathOptimization/maps/points/points.csv"
        )

        # get local position
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.pose_callback, 10
        )

        self.cmd_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        
        # Lookahead parameters
        # self.lookahead_min = 1.0  # Minimum lookahead distance
        # self.lookahead_max = 3.0  # Maximum lookahead distance
        self.lookahead = 1.0
        self.lookahead_gain = 1.0  # Gain for speed-based lookahead adjustment
        
        self.accel_max  = 2.5
        self.vel_max = 2.0
        self.current_pose = None
        self.lap_count = 0
        self.last_closest_idx = 0

    def load_waypoints(self, csv_file: str):
        """
        Load a CSV of x,y waypoints and convert to a NumPy structured array
        of PathPoints with fields: x, y, heading, distance.
        """
        # Load raw x,y data
        data = np.loadtxt(csv_file, delimiter=",", skiprows=1)

        xs = data[:, 0]
        ys = data[:, 1]

        # Add the first point at the end to create a loop
        xs = np.append(xs, xs[0])
        ys = np.append(ys, ys[0])

        # Compute segment deltas
        dx = np.diff(xs)
        dy = np.diff(ys)

        # Heading = arctan2(dy, dx); repeat last heading for final point
        headings = np.arctan2(dy, dx)
        headings = np.concatenate([headings, headings[-1:]])

        # Cumulative distance along the path
        segment_lengths = np.hypot(dx, dy)
        distances = np.concatenate([[0.0], np.cumsum(segment_lengths)])

        # Define structured dtype for PathPoint
        path_dtype = np.dtype(
            [
                ("x", np.float64),
                ("y", np.float64),
                ("heading", np.float64),
                ("distance", np.float64),
            ]
        )

        # Populate structured array
        path_points = np.zeros(len(xs), dtype=path_dtype)
        path_points["x"] = xs
        path_points["y"] = ys
        path_points["heading"] = headings
        path_points["distance"] = distances

        return path_points

    def pose_callback(self, msg):
        """Callback for AMCL pose updates."""
        self.current_pose = msg

        # If we have a valid pose and waypoints, calculate control
        if self.current_pose is not None and len(self.waypoints) > 0:
            self.drive_callback()

    def find_closest_point(self):
        """Finds the closest waypoint to the current position of the car."""
        x0 = self.current_pose.pose.pose.position.x
        y0 = self.current_pose.pose.pose.position.y

        dx = self.waypoints["x"] - x0
        dy = self.waypoints["y"] - y0

        dists = np.hypot(dx, dy)
        closest_idx = int(np.argmin(dists))
        
        # Check if we've completed a lap
        if closest_idx < self.last_closest_idx and self.last_closest_idx - closest_idx > len(self.waypoints) // 2:
            self.lap_count += 1
            self.get_logger().info(f"Completed lap {self.lap_count}")
        
        self.last_closest_idx = closest_idx
        return closest_idx

    def find_goal_point(self, closest_idx):
        """Find the next waypoint at a variable lookahead distance away from the position of the car"""
        x0 = self.current_pose.pose.pose.position.x
        y0 = self.current_pose.pose.pose.position.y
        xs = self.waypoints["x"]
        ys = self.waypoints["y"]

        # Search forward from the closest point
        for idx in range(closest_idx, len(self.waypoints)):
            dx = xs[idx] - x0
            dy = ys[idx] - y0
            if np.hypot(dx, dy) >= self.lookahead:
                return idx

        # If none found, fall back to the closest point
        return closest_idx

    def drive_callback(self):
        """Using the local pose and waypoints, drive to the next goal point"""

        # Get current position and orientation
        current_pos = self.current_pose.pose.pose.position
        current_orient = self.current_pose.pose.pose.orientation
        x, y = current_pos.x, current_pos.y
        qx, qy, qz, qw = (
            current_orient.x,
            current_orient.y,
            current_orient.z,
            current_orient.w,
        )
        
        # find the path point closest to the vehicle
        closest_idx = self.find_closest_point()

        # find the goal point
        goal_idx = self.find_goal_point(closest_idx)
        goal_point = self.waypoints[goal_idx]

        # transform the goal point to the vehicle coordinates
        dx = goal_point["x"] - x
        dy = goal_point["y"] - y
        current_yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

        # goal point's local coordinates
        x_l = dx * math.cos(current_yaw) + dy * math.sin(current_yaw)
        y_l = -dx * math.sin(current_yaw) + dy * math.cos(current_yaw)

        # calculate curvature
        l_sq = x_l**2 + y_l**2
        if l_sq == 0:
            kappa = 0.0
        else:
            kappa = 2.0 * y_l / l_sq
            
        # compute local speed profile
        abs_k = abs(kappa) if abs(kappa) > 1e-6 else 1e-6
        v_curve = math.sqrt(self.accel_max / abs_k)

        # cap at max velocity
        v_out = min(v_curve, self.vel_max)
        
        # Store current speed for lookahead calculation
        self.current_speed = v_out
        heading = math.atan2(2 * y_l, x_l)
        self.get_logger().info(f"Speed: {v_out}, Heading: {heading}")
        # drive the car        
        cmd = AckermannDriveStamped()
        cmd.drive.speed = v_out
        cmd.drive.steering_angle = heading # Proper steering angle calculation
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
