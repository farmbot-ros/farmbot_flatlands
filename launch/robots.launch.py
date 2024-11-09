# multi_robot_launch.py

import os
import yaml

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, GroupAction
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to the simulation.yaml file
    pkg_share = get_package_share_directory('farmbot_flatlands')  # Use your package name
    config_file = os.path.join(pkg_share, 'config', 'simulation.yaml')

    # Load the YAML file
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)

    robots = config.get('robots', [])

    launch_description = LaunchDescription()

    for robot in robots:
        namespace = robot.get('namespace', 'robot_default')
        initial_pose = robot.get('initial_pose', {})
        initial_x = initial_pose.get('x', 0.0)
        initial_y = initial_pose.get('y', 0.0)
        initial_theta = initial_pose.get('theta', 0.0)

        # Launch the MobileRobotSimulator node within the specified namespace
        robot_node = Node(
            package='farmbot_flatlands',  # Use your package name
            executable='robot',  # Use your executable name
            name='mobile_robot_simulator',
            namespace=namespace,
            parameters=[
                {'publish_rate': 10.0},
                {'velocity_topic': 'cmd_vel'},
                {'odometry_topic': 'pose'},
                {'marker_topic': 'robopose'},
                {'odom_frame': 'odom'},
                {'base_frame': 'base_link'},
                {'initial_x' : initial_x},
                {'initial_y' : initial_y},
                {'initial_theta' : initial_theta},
                {'use_sim_time': True}
            ],
            output='screen'
        )

        launch_description.add_action(robot_node)

        initial_pose_args = [ '0', '0', '0', '0', '0', '0', '1', 'map', 'odom' ]

        # Create the ExecuteProcess action without the namespace parameter
        publish_static_tf = ExecuteProcess(
            cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher'] + initial_pose_args,
            output='screen'
        )

        # Wrap the ExecuteProcess in a GroupAction with PushRosNamespace
        static_tf_group = GroupAction(
            actions=[
                PushRosNamespace(namespace),
                publish_static_tf
            ]
        )

        # Delay the static transform publishing slightly to ensure the node is ready
        delayed_static_tf = TimerAction(
            period=1.0,  # 1 second delay
            actions=[static_tf_group]
        )

        launch_description.add_action(delayed_static_tf)

    return launch_description
