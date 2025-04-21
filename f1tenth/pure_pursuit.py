#!/usr/bin/env python3
import csv
import rclpy
from rclpy.node import Node
import numpy as np
import json
import math
from geometry_msgs.msg import PoseWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, Quaternion
from rclpy.duration import Duration

class PathPoint():
    def __init__(self, x, y, heading, curvature, distance):
        # global pose
        self.x = x
        self.y = y
        self.heading = heading
        self.curvature = curvature
        self.distance = distance

class PurePursuitController(Node):
    def __init__(self):
        # Initialize ROS 2 node
        super().__init__('pure_pursuit_controller')
        
        # Load waypoints from CSV file
        # this looks like pure odom data, but we likely want to bring this in from a trajectory planner
        # so the planner should give up a csv/json of the appropriate format to place into this
        self.waypoints = self.load_waypoints('/home/nvidia/team2TEMP/src/f1tenth/VipPathOptimization/maps/points/points.csv') 
        self.current_waypoint_index = 0
        self.path = PathPoint()
        # Pure pursuit parameters - defined directly in the code
        self.lookahead_distance = 1.0  # meters
        self.max_speed = 0.5  # m/s
        self.max_steering_angle = 0.5  # radians (approximately 28.6 degrees)
        self.wheel_base = 0.5  # meters (distance between front and rear axles)
        self.waypoint_threshold = 0.5  # meters
        
        # Initialize TF buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # get local position
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, 
            '/amcl_pose', 
            self.pose_callback, 
            10
        )
        
        # Publishers
        self.cmd_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        
        # Current pose
        self.current_pose = None
        
        self.get_logger().info("Pure Pursuit Controller initialized")
        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints")
        
    # def load_waypoints(self, filename):
    #     """Load waypoints from JSON file."""
    #     try:
    #         with open(filename, 'r') as f:
    #             waypoints = json.load(f)
    #         self.get_logger().info(f"Successfully loaded waypoints from {filename}")
    #         return waypoints
    #     except Exception as e:
    #         self.get_logger().error(f"Failed to load waypoints: {e}")
    #         return []
    
    def compute_curvature(self, x, y, ds=None):
        """
        Compute curvature kappa[i] = (x' y'' - y' x'') / (x'^2 + y'^2)^(3/2)

        Parameters
        ----------
        x : array-like, shape (N,)
            x-coordinates of the path.
        y : array-like, shape (N,)
            y-coordinates of the path.
        ds : float, optional
            Spacing between points. If None, it will be estimated as the mean
            Euclidean distance between consecutive points.

        Returns
        -------
        kappa : ndarray, shape (N,)
            Curvature at each point.
        """
        x = np.asarray(x, dtype=float)
        y = np.asarray(y, dtype=float)
        N = len(x)
        if N < 3:
            raise ValueError("Need at least 3 points to compute curvature.")

        # estimate ds if not provided
        if ds is None:
            dx_seg = np.diff(x)
            dy_seg = np.diff(y)
            ds = np.mean(np.hypot(dx_seg, dy_seg))

        # allocate derivative arrays
        x1 = np.empty(N)
        y1 = np.empty(N)
        x2 = np.empty(N)
        y2 = np.empty(N)

        # first derivative: forward/backward at ends, central inside
        x1[0]   = (x[1]   - x[0])   / ds
        y1[0]   = (y[1]   - y[0])   / ds
        x1[-1]  = (x[-1]  - x[-2])  / ds
        y1[-1]  = (y[-1]  - y[-2])  / ds
        x1[1:-1] = (x[2:] - x[:-2]) / (2*ds)
        y1[1:-1] = (y[2:] - y[:-2]) / (2*ds)

        # second derivative: zero at ends (or you could forward/backwards)
        x2[0]    = 0.0
        y2[0]    = 0.0
        x2[-1]   = 0.0
        y2[-1]   = 0.0
        x2[1:-1] = (x[2:] - 2*x[1:-1] + x[:-2]) / (ds*ds)
        y2[1:-1] = (y[2:] - 2*y[1:-1] + y[:-2]) / (ds*ds)

        # curvature formula
        numerator   = x1 * y2 - y1 * x2
        denominator = (x1**2 + y1**2)**1.5
        kappa = numerator / denominator

        return kappa
    
    def load_waypoints(self, filename):
        """
        Load waypoints from a CSV with columns 'x','y' (header row required).
        Returns a list of (x, y) tuples.
        """
        waypoints = []
        
        try:
            with open('your_file.csv', newline='') as csvfile:
                row = list(csv.reader(csvfile))
                arrX = []
                arrY = []
                for i in range(len(row) - 1):
                    arrX.append([row[i][0]])
                    arrY.append([row[i][1]])
                    
                    x1 = row[i][0]
                    x2 = row[i + 1][0]
                    y1 = row[i][1]
                    y2 = row[i + 1][1]
                    
                    heading = math.atan2(y2-y1, x2-x1)
                    curvature = self.compute_curvature(arrX, arrY)
                    distance = np.hypot(y2-y1, x2-x1)
                    
                    waypoints.append(PathPoint(x1,y1,heading, curvature, distance))
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints: {e}")
        
        return waypoints
        
        # try:
        #     with open(filename, 'r') as csvfile:
        #         reader = csv.DictReader(csvfile)
        #         for row in reader:
        #             # cast to float and append as a tuple
        #             # instead append a PathPoint object
        #             # calculate heading
        #             waypoints.append((float(row['A']), float(row['B'])))
        #     self.get_logger().info(f"Loaded {len(waypoints)} waypoints from {filename}")
        # except Exception as e:
        #     self.get_logger().error(f"Failed to load waypoints: {e}")
        # return waypoints

    def pose_callback(self, msg):
        """Callback for AMCL pose updates."""
        self.current_pose = msg
        
        # If we have a valid pose and waypoints, calculate control
        if self.current_pose and self.waypoints:
            self.calculate_control()
    
    def calculate_control(self):
        """Calculate control commands using pure pursuit."""
        # Get current position and orientation
        current_pos = self.current_pose.pose.pose.position
        current_orient = self.current_pose.pose.pose.orientation
        
        ## determine the path point closest to the vehicle
        for point in self.waypoints:
            
        
        # Convert quaternion to euler angles (yaw)
        # For a quaternion q = [x, y, z, w], yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        x, y, z, w = current_orient.x, current_orient.y, current_orient.z, current_orient.w
        current_yaw = math.atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z))
        
        # Check if we've reached the current waypoint
        current_waypoint = self.waypoints[self.current_waypoint_index]
        waypoint_pos = current_waypoint['pose']['pose']['position']
        
        distance_to_waypoint = math.sqrt(
            (current_pos.x - waypoint_pos['x'])**2 + 
            (current_pos.y - waypoint_pos['y'])**2
        )
        
        # If we're close enough to the current waypoint, move to the next one
        if distance_to_waypoint < self.waypoint_threshold:
            self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.waypoints)
            self.get_logger().info(f"Reached waypoint {self.current_waypoint_index-1}, moving to next waypoint")
            
            # If we've completed the path, stop
            if self.current_waypoint_index == 0:
                self.get_logger().info("Path completed!")
                self.stop_robot()
                return
        
        # Find the lookahead point
        lookahead_point = self.find_lookahead_point(
            current_pos.x, current_pos.y, current_yaw
        )
        
        if lookahead_point is None:
            self.get_logger().warn("No lookahead point found, stopping robot")
            self.stop_robot()
            return
        
        # Calculate control commands
        cmd = AckermannDriveStamped()
        
        # Calculate steering angle
        # Pure pursuit formula for Ackermann steering: δ = arctan(2L*sin(α)/d)
        # where L is the wheelbase, α is the angle to the lookahead point, and d is the lookahead distance
        dx = lookahead_point[0] - current_pos.x
        dy = lookahead_point[1] - current_pos.y
        
        # Angle to lookahead point
        alpha = math.atan2(dy, dx) - current_yaw
        
        # Normalize angle to [-pi, pi]
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))
        
        # Calculate steering angle using Ackermann geometry
        # For small angles, tan(δ) ≈ L/R where R is the turning radius
        # R = d/(2*sin(α)) from pure pursuit geometry
        # Therefore, δ = arctan(2L*sin(α)/d)
        steering_angle = math.atan2(2.0 * self.wheel_base * math.sin(alpha), self.lookahead_distance)
        
        # Limit steering angle
        steering_angle = max(min(steering_angle, self.max_steering_angle), -self.max_steering_angle)
        
        # Set speed (constant for now)
        cmd.drive.speed = self.max_speed
        
        # Set steering angle
        cmd.drive.steering_angle = steering_angle
        
        # Set acceleration and jerk to 0 (constant speed)
        cmd.drive.acceleration = 0.0
        cmd.drive.jerk = 0.0
        
        # Publish command
        self.cmd_pub.publish(cmd)
        
        # Log control values
        self.get_logger().debug(f"Distance to waypoint: {distance_to_waypoint:.2f}m, Steering angle: {steering_angle:.2f}rad")
    
    def find_lookahead_point(self, x, y, yaw):
        """Find the lookahead point on the path."""
        # Get the current waypoint and next waypoint
        current_waypoint = self.waypoints[self.current_waypoint_index]
        next_waypoint_index = (self.current_waypoint_index + 1) % len(self.waypoints)
        next_waypoint = self.waypoints[next_waypoint_index]
        
        # Extract positions
        current_pos = current_waypoint['pose']['pose']['position']
        next_pos = next_waypoint['pose']['pose']['position']
        
        # Line segment between current and next waypoint
        line_start = np.array([current_pos['x'], current_pos['y']])
        line_end = np.array([next_pos['x'], next_pos['y']])
        robot_pos = np.array([x, y])
        
        # Vector from line start to line end
        line_vec = line_end - line_start
        line_length = np.linalg.norm(line_vec)
        
        if line_length == 0:
            return None
        
        # Normalized direction vector
        line_dir = line_vec / line_length
        
        # Vector from line start to robot
        robot_vec = robot_pos - line_start
        
        # Project robot position onto line
        projection = np.dot(robot_vec, line_dir)
        
        # Clamp projection to line segment
        projection = max(0, min(projection, line_length))
        
        # Point on line closest to robot
        closest_point = line_start + projection * line_dir
        
        # Vector from robot to closest point
        to_closest = closest_point - robot_pos
        to_closest_length = np.linalg.norm(to_closest)
        
        # If robot is beyond the line segment, use the next waypoint
        if projection >= line_length:
            return (next_pos['x'], next_pos['y'])
        
        # Calculate lookahead point
        # Move along the line in the direction of travel
        lookahead_dist = self.lookahead_distance
        
        # If we're close to the end of the line segment, use the next waypoint
        if projection + lookahead_dist > line_length:
            return (next_pos['x'], next_pos['y'])
        
        # Otherwise, move along the current line segment
        lookahead_point = line_start + (projection + lookahead_dist) * line_dir
        
        return (lookahead_point[0], lookahead_point[1])
    
    def stop_robot(self):
        """Stop the robot."""
        cmd = AckermannDriveStamped()
        cmd.drive.speed = 0.0
        cmd.drive.steering_angle = 0.0
        cmd.drive.acceleration = 0.0
        cmd.drive.jerk = 0.0
        self.cmd_pub.publish(cmd)
    
    def run(self):
        """Run the controller."""
        rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    controller = PurePursuitController()
    controller.run()
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 
