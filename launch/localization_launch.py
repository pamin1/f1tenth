import launch
from launch.actions import TimerAction, ExecuteProcess
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, LogInfo
import os

def generate_launch_description():
    joy_teleop_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'joy_teleop.yaml'
    )
    vesc_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'vesc.yaml'
    )
    sensors_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'sensors.yaml'
    )
    mux_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'mux.yaml'
    )

    joy_la = DeclareLaunchArgument(
        'joy_config',
        default_value=joy_teleop_config,
        description='Descriptions for joy and joy_teleop configs')
    vesc_la = DeclareLaunchArgument(
        'vesc_config',
        default_value=vesc_config,
        description='Descriptions for vesc configs')
    sensors_la = DeclareLaunchArgument(
        'sensors_config',
        default_value=sensors_config,
        description='Descriptions for sensor configs')
    mux_la = DeclareLaunchArgument(
        'mux_config',
        default_value=mux_config,
        description='Descriptions for ackermann mux configs')

    ld = LaunchDescription([joy_la, vesc_la, sensors_la, mux_la])

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        parameters=[LaunchConfiguration('joy_config')]
    )
    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[LaunchConfiguration('joy_config')]
    )
    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    throttle_interpolator_node = Node(
        package='f1tenth_stack',
        executable='throttle_interpolator',
        name='throttle_interpolator',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    urg_node = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node',
        parameters=[LaunchConfiguration('sensors_config')]
    )
    ackermann_mux_node = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        parameters=[LaunchConfiguration('mux_config')],
        remappings=[('ackermann_cmd_out', 'ackermann_drive')]
    )
    
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_laser',
        arguments=['0.27', '0.0', '0.11', '0.0', '0.0', '0.0', 'base_link', 'laser']
    )
    
    static_tf_node2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_odom_to_baselink',
        arguments=['0.27', '0.0', '0.11', '0.0', '0.0', '0.0', 'odom', 'base_link']
    )

    static_tf_node3 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0.27', '0.0', '0.11', '0.0', '0.0', '0.0', 'map', 'odom']
    )

    dynamic_tf_publisher_node = Node(
        package='reactive_racing',
        executable='dynamic_transform_publisher',  # Correct the executable name here if needed
        name='dynamic_transform_publisher',
        output="screen",
    )

    ctrl_node = Node(
        package='f1tenth',
        executable='following',
        name='control',
        output="screen",
    )

    slam = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'
    )

    # Launch the map server node as a lifecycle node with a specified YAML file.
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='mapserver',  # Name must match the lifecycle bringup argument.
        output='screen',
        parameters=[{'yaml_filename': '/home/nvidia/my_map.yaml'}]
    )

    # Launch the AMCL node with the given parameter.
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{'base_frame_id': 'base_link',
                     'set_initial_pose': True,
                     'initial_pose': {'x': 2.47508, 'y': 6.04826, 'z': 0.0,},
                     'odom_frame_id': 'odom'}]
    )

    ld.add_action(map_server_node)
    ld.add_action(amcl_node)

    # Bring up both nodes' lifecycle interfaces using a single lifecycle_bringup command.
    lifecycle_bringup = ExecuteProcess(
        cmd=['ros2', 'run', 'nav2_util', 'lifecycle_bringup', 'mapserver', 'amcl'],
        output='screen'
    )

    # finalize
    ld.add_action(joy_node)
    ld.add_action(joy_teleop_node)
    ld.add_action(ackermann_to_vesc_node)
    ld.add_action(vesc_to_odom_node)
    ld.add_action(vesc_driver_node)
    # ld.add_action(throttle_interpolator_node)
    ld.add_action(urg_node)
    ld.add_action(ackermann_mux_node)
    ld.add_action(static_tf_node)
    ld.add_action(static_tf_node2)
    ld.add_action(static_tf_node3)
    ld.add_action(lifecycle_bringup)
    # ld.add_action(map)
    # ld.add_action(activate_map)
    # ld.add_action(slam_include)
    # ld.add_action(dynamic_tf_publisher_node) 
    # ld.add_action(teleop2drive)
    # ld.add_action(TimerAction(
    #    period=1.0,
    #    actions=[ctrl_node]
    # ))
    return ld
