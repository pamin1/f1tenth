#!/usr/bin/env python
# Akhil Kothapalli January 2025


from ackermann_msgs.msg import AckermannDriveStamped
import itertools
import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import LaserScan
import numpy as np
import time
import math

noForward = False

# vehicle parameters
MAX_THROTTLE = 1.0
TURN_ANGLE = math.radians(45)
#behavoir param
TOLERANCE_MULT = 3
MIN_DISTANCE_TO_WALL = 0.6



if noForward:
    MAX_THROTTLE = -0.1




def ellipse_transform(x, b, a=1.7):
    return b * np.sqrt(1 - (x / a)**2)


class WallAvoid(Node):
    def __init__(self, lidar_gap_following=True):
        super().__init__('wall_avoid')
        self.lidar_gap_following = lidar_gap_following

        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=rclpy.qos.DurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE)

        if lidar_gap_following:
            self.lidar_subscription = self.create_subscription(
                LaserScan,
                '/scan',
                self.lidar_callback,
                qos_policy)
        else:
            assert self.lidar_gap_following, "lidar_gap_following=False is not implemented"
            pass
        self.control_publisher = self.create_publisher(
            AckermannDriveStamped,
            '/drive', 
            qos_policy)

    def lidar_callback(self, msg):
        print("lidar_callback")
        
        smallest = min(msg.ranges)
        min_index = msg.ranges.index(smallest)
        angle = msg.angle_min + min_index * msg.angle_increment
        print(f"Smallest Possible Angle {msg.angle_min}")
        print(f"smallest {smallest}")
        print(f"angle {angle}")
        #print(f"msg.angle_min{msg.angle_min}")
        tolerance = MIN_DISTANCE_TO_WALL
        #tolerance = MIN_DISTANCE_TO_WALL * ellipse_transform(abs(angle), 2, abs(msg.angle_min))
        print(f"Tolerance {tolerance}")
        #if -0.26 < angle < 0.26 and smallest < 0.4:
        #    print("Going slight turn")
        #    self.move(TURN_ANGLE, MAX_THROTTLE)
        
        if smallest > tolerance:
            print("Going forward")
            self.goForward()    
        else:
            if angle < 0:
                print("Going left")
                self.goLeft()
            
            else:
                print("Going Right")
                self.goRight()

    def goForward(self):
        self.move(FORWARD_TURN_BIAS, MAX_THROTTLE)
    
    def goRight(self):
        self.move(-TURN_ANGLE, MAX_THROTTLE)
    
    def goLeft(self):
        self.move(TURN_ANGLE, MAX_THROTTLE)




    def move(self, turnAngle, throttle):
        print(f"Throttle {throttle}, Turn Angle {turnAngle}")
        ack_msg = AckermannDriveStamped()
        ack_msg.drive.steering_angle = turnAngle
        ack_msg.drive.speed = float(throttle)
        ack_msg.header.stamp = self.get_clock().now().to_msg()
        ack_msg.header.frame_id = 'base_link'
        self.control_publisher.publish(ack_msg)





def main(args=None):
    rclpy.init(args=args)
    node = WallAvoid()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

