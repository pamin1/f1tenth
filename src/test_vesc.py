#!/usr/bin/env python
# Akhil Kothapalli January 2025

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from ackermann_msgs.msg import AckermannDriveStamped
import math

class AutoDriveNode(Node):
    def __init__(self):
        super().__init__('auto_drive')
        qos_policy = 1
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/drive', qos_policy)
        timer_period = 0.1  
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed = 0.0  
        msg.drive.steering_angle = math.pi/3  
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    auto_drive_node = AutoDriveNode()
    rclpy.spin(auto_drive_node)
    auto_drive_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
