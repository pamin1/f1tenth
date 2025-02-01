from launch import LaunchDescription
from launch_ros.actions import Node
import os
from glob import glob

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mpcc',
            namespace='ego_agent',
            executable='ego_agent',            
        ),
        Node (
            package='mpcc',
            namespace='opp_follower',
            executable='following',
            parameters=[
                {'steering_kP' : 0.0},
                {'steering_kI' : 0.01},
                {'steering_kD' : 0.006},
                {'throttle_kP' : 0.},
                {'throttle_kI' : 0.2},
                {'throttle_kD' : 0.005}
            ]
        )
    ])