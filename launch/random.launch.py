# multi_robot_launch.py
import os
import random
import math
import yaml

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
    default_base_latitude = datum[0] if len(datum) > 0 else 0.0
    default_base_longitude = datum[1] if len(datum) > 1 else 0.0
    default_base_altitude = datum[2] if len(datum) > 2 else 0.0

    default_num_robots = global_params.get('num_robots', 1)
    default_max_distance = global_params.get('max_distance', 200.0)
    default_min_distance = global_params.get('min_distance', 100.0)

    # Declare launch arguments with defaults from the config file
    base_latitude_arg = DeclareLaunchArgument(
        'base_latitude',
        default_value=str(default_base_latitude),
        description='Base latitude for robot spawning'
    )

    base_longitude_arg = DeclareLaunchArgument(
        'base_longitude',
        default_value=str(default_base_longitude),
        description='Base longitude for robot spawning'
    )

    base_altitude_arg = DeclareLaunchArgument(
        'base_altitude',
        default_value=str(default_base_altitude),
        description='Base altitude for robot spawning'
    )

    num_robots_arg = DeclareLaunchArgument(
        'num_robots',
        default_value=str(default_num_robots),
        description='Number of robots to spawn'
    )

    max_distance_arg = DeclareLaunchArgument(
        'max_distance',
        default_value=str(default_max_distance),
        description='Maximum random distance from base position (in meters)'
    )

    min_distance_arg = DeclareLaunchArgument(
        'min_distance',
        default_value=str(default_min_distance),
        description='Minimum random distance from base position (in meters)'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the launch arguments to the launch description
    ld.add_action(base_latitude_arg)
    ld.add_action(base_longitude_arg)
    ld.add_action(base_altitude_arg)
    ld.add_action(num_robots_arg)
    ld.add_action(max_distance_arg)
    ld.add_action(min_distance_arg)

    # Use an OpaqueFunction to process the arguments and generate the nodes
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld

def launch_setup(context, *args, **kwargs):
    # Retrieve the launch configuration variables
    base_latitude = float(LaunchConfiguration('base_latitude').perform(context))
    base_longitude = float(LaunchConfiguration('base_longitude').perform(context))
    base_altitude = float(LaunchConfiguration('base_altitude').perform(context))
    num_robots = int(LaunchConfiguration('num_robots').perform(context))
    max_distance = float(LaunchConfiguration('max_distance').perform(context))
    min_distance = float(LaunchConfiguration('min_distance').perform(context))

    actions = []

    for i in range(num_robots):
        # Generate random offsets within max_distance
        distance = random.uniform(min_distance, max_distance)
        angle = random.uniform(0, 2 * math.pi)

        # Approximate conversion from meters to degrees
        delta_latitude = (distance * math.cos(angle)) / 111000  # Degrees latitude per meter
        delta_longitude = (distance * math.sin(angle)) / (111000 * math.cos(math.radians(base_latitude)))  # Degrees longitude per meter

        robot_latitude = base_latitude + delta_latitude
        robot_longitude = base_longitude + delta_longitude
        # robot_altitude = base_altitude+random.uniform(0.5,1.5)
        robot_altitude = base_altitude

        namespace = f'robot{i}'
        heading = random.uniform(0, 360)
        print(f'Robot {i} at {robot_latitude}, {robot_longitude}, {robot_altitude} with heading {heading}')

        # Create the MobileRobotSimulator node
        robot_node = Node(
            package='farmbot_flatlands',
            executable='simulator',
            name='simulator',
            namespace=namespace,
            parameters=[
                {'publish_rate': 10.0},
                {'velocity_topic': 'cmd_vel'},
                {'odometry_topic': 'loc/odom'},
                {'reference_topic': 'loc/ref/geo'},
                {'position_topic': 'gps_center'},
                {'heading_topic': 'heading'},
                {'imu_topic': 'imu/data'},
                {'latitude': robot_latitude},
                {'longitude': robot_longitude},
                {'altitude': robot_altitude},
                {'heading': heading}
            ],
            output='screen'
        )

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

        actions.append(robot_node)
        actions.append(visualize_node)

    return actions
