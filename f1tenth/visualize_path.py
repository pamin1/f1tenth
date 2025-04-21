#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import csv

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        # 1) Publisher on "/path"
        self.pub = self.create_publisher(Path, 'path', 10)
        # 2) Load CSV only once at startup
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'     # <- choose your fixed frame
        self._load_csv('path.csv')

        # 3) Publish at 1 Hz (or whatever you like)
        self.create_timer(1.0, self.publish_path)

    def _load_csv(self, filename):
        poses = []
        with open(filename, newline='') as f:
            reader = csv.reader(f)
            for row in reader:
                x, y = float(row[0]), float(row[1])
                pose = PoseStamped()
                pose.header.frame_id = self.path_msg.header.frame_id
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0.0
                # no orientation info in CSV -> make a valid unit quaternion
                pose.pose.orientation.w = 1.0
                poses.append(pose)
        self.path_msg.poses = poses

    def publish_path(self):
        now = self.get_clock().now().to_msg()
        self.path_msg.header.stamp = now
        for p in self.path_msg.poses:
            p.header.stamp = now
        self.pub.publish(self.path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
