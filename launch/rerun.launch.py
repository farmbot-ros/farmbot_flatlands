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
    ld = LaunchDescription()
    pkg_share = get_package_share_directory('farmbot_flatlands')
    config_file = os.path.join(pkg_share, 'config', 'simulation.yaml')

    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)

    num_robots_param = config.get('global', {}).get('ros__parameters', {}).get('num_robots', 1)

    num_robots_arg = DeclareLaunchArgument(
        'num_robots',
        default_value=str(num_robots_param),
        description='Number of robots to spawn'
    )
    ld.add_action(num_robots_arg)

    tcp_arg = DeclareLaunchArgument(
        'tcp',
        default_value='127.0.0.0:9876',
        description='TCP address to connect to the rerun server'
    )
    ld.add_action(tcp_arg)


    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld


def launch_setup(context, *args, **kwargs):
    num_robots = int(LaunchConfiguration('num_robots').perform(context))
    tcp = str(LaunchConfiguration('tcp').perform(context))

    pgk_share = get_package_share_directory('farmbot_holodeck')
    launch_file = os.path.join(pgk_share, 'launch', 'rerun.launch.py')

    actions = []

    for i in range(num_robots):
        namespace = f'robot{i}'
        navigation_launch = GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_file),
                launch_arguments={
                    'namespace': namespace,
                    'tcp': tcp
                }.items()
            )
        ])
        actions.append(navigation_launch)


    return actions
