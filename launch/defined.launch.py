# multi_robot_launch.py
import os
import yaml

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, GroupAction
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to the simulation.yaml file
    pkg_share = get_package_share_directory('farmbot_flatlands')
    config_file = os.path.join(pkg_share, 'config', 'simulation.yaml')

    # Load the YAML file
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)

    robots = config.get('robots', [])

    launch_description = LaunchDescription()

    for robot in robots:
        namespace = robot.get('namespace', 'robot_default')
        initial_pose = robot.get('initial_pose', {})
        latitude = initial_pose.get('latitude', 0)
        longitude = initial_pose.get('longitude', 0)
        heading = initial_pose.get('heading', 0)

        # Launch the MobileRobotSimulator node within the specified namespace
        robot_node = Node(
            package='farmbot_flatlands',
            executable='robot',
            name='mobile_robot_simulator',
            namespace=namespace,
            parameters=[
                {'publish_rate': 10.0},
                {'velocity_topic': 'cmd_vel'},
                {'odometry_topic': 'loc/odom'},
                {'reference_topic': 'loc/ref/geo'},
                {'position_topic': 'gps_center'},
                {'heading_topic': 'heading'},
                {'latitude': latitude},
                {'longitude': longitude},
                {'heading': heading}
            ],
            output='screen'
        )

        launch_description.add_action(robot_node)

        visualize_node = Node(
            package='farmbot_flatlands',
            executable='visualize',
            name='visualize_robot',
            namespace=namespace,
            parameters=[
                {'robot_name': namespace},
                {'robot_color': 'blue'},
                {'robot_width': 0.5},
                {'robot_length': 0.5},
                {'robot_height': 0.5},
            ],
            output='screen'
        )

        launch_description.add_action(visualize_node)


    return launch_description
