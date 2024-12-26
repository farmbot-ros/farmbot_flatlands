# multi_localization_launch.py
import os
import yaml

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package share directory and config file path
    pkg_share = get_package_share_directory('farmbot_flatlands')
    config_file = os.path.join(pkg_share, 'config', 'simulation.yaml')

    # Load the YAML configuration file
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)

    # Extract the number of robots from the config file
    num_robots_param = config.get('global', {}).get('ros__parameters', {}).get('num_robots', 1)

    num_robots_arg = DeclareLaunchArgument(
        'num_robots',
        default_value=str(num_robots_param),
        description='Number of robots to spawn'
    )

    # Path to the localization.launch.py file

    # Create the main launch description
    ld = LaunchDescription()
    ld.add_action(num_robots_arg)

    # Add ann OpaqueFunction to the launch description
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld


def launch_setup(context, *args, **kwargs):
    num_robots = int(LaunchConfiguration('num_robots').perform(context))

    pkg_share_localization = get_package_share_directory('farmbot_polestar')
    localization_launch_file = os.path.join(pkg_share_localization, 'launch', 'localization.launch.py')

    actions = []

    for i in range(num_robots):
        namespace = f'robot{i}'
        # GroupAction to launch the localization.launch.py file under the given namespace
        robot_launch = GroupAction([
            # PushRosNamespace(namespace),  # Push the namespace for this robot
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(localization_launch_file),
                launch_arguments={
                    'namespace': namespace,
                    'autodatum': 'datum'
                }.items()
            )
        ])

        actions.append(robot_launch)

    return actions
