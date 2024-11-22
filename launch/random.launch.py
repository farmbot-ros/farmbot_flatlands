import os
import random
import math
import yaml
from functools import partial  # Import partial

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package share directory and config file path
    pkg_share = get_package_share_directory('farmbot_flatlands')
    config_file = os.path.join(pkg_share, 'config', 'simulation.yaml')

    # Load the YAML configuration file
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)

    # Extract default values from the config file
    # Adjusting to the structure of your simulation.yaml
    global_params = config.get('global', {}).get('ros__parameters', {})

    datum = global_params.get('datum', [0.0, 0.0, 0.0])
    default_num_robots = global_params.get('num_robots', 1)

    num_robots_arg = DeclareLaunchArgument(
        'num_robots',
        default_value=str(default_num_robots),
        description='Number of robots to spawn'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(num_robots_arg)

    # Use functools.partial to pass datum to launch_setup
    ld.add_action(OpaqueFunction(function=partial(launch_setup, datum=datum)))

    return ld

def launch_setup(context, datum, *args, **kwargs):
    # Retrieve the launch configuration variables
    num_robots = int(LaunchConfiguration('num_robots').perform(context))
    actions = []
    # Create the MobileRobotSimulator node
    robot_node = Node(
        package='farmbot_flatlands',
        executable='simulator',
        name='simulator',
        # namespace=namespace,
        parameters=[{
            'publish_rate': 10.0,
            'datum': datum,  # Use the passed datum
            'max_linear_accel': 0.7,
            'max_angular_accel': 0.7,
            'num_robots': num_robots,
        }],
        output='screen'
    )
    actions.append(robot_node)

    for i in range(num_robots):
        namespace = f'robot{i}'

        # Create the visualization node
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

        actions.append(visualize_node)

    return actions
