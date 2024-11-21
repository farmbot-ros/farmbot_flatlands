# send_waypoints_launch.py
import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_mission_goal(robot_id, num_robots=1):
    """
    Generate a mission goal with offsets based on the robot's ID.

    Args:
        robot_id (int): The identifier for the robot (e.g., 0 for robot0).

    Returns:
        str: A YAML-formatted string representing the mission goal.
    """
    # Define the base waypoints
    length = 20.0
    base_poses = [
        {"position": {"x": 0.0, "y": 0.0+float(robot_id), "z": 0.0}},
        {"position": {"x": length, "y": 0.0+float(robot_id), "z": 0.0}},
        {"position": {"x": length, "y": num_robots+float(robot_id), "z": 0.0}},
        {"position": {"x": 0.0, "y": num_robots+float(robot_id), "z": 0.0}},
        {"position": {"x": 0.0, "y": (num_robots*2)+float(robot_id), "z": 0.0}},
        {"position": {"x": length, "y": (num_robots*2)+float(robot_id), "z": 0.0}},
        {"position": {"x": length, "y": (num_robots*3)+float(robot_id), "z": 0.0}},
        {"position": {"x": 0.0, "y": (num_robots*3)+float(robot_id), "z": 0.0}},
        {"position": {"x": 0.0, "y": (num_robots*4)+float(robot_id), "z": 0.0}},
        {"position": {"x": length, "y": (num_robots*4)+float(robot_id), "z": 0.0}},
    ]

    # Apply offsets to each pose
    poses = []
    for pose in base_poses:
        new_pose = {
            "pose": {
                "position": {
                    "x": pose["position"]["x"],
                    "y": pose["position"]["y"],
                    "z": pose["position"]["z"]
                }
            }
        }
        poses.append(new_pose)

    # Construct the mission goal YAML string
    mission_goal = "mission:\n  poses:\n"
    for p in poses:
        mission_goal += f"    - pose:\n        position: {{x: {p['pose']['position']['x']}, y: {p['pose']['position']['y']}, z: {p['pose']['position']['z']}}}\n"

    return mission_goal

def launch_setup(context, *args, **kwargs):
    # Retrieve the number of robots from the launch configuration
    num_robots = int(LaunchConfiguration('num_robots').perform(context))
    # Path to the configuration file
    pkg_share = get_package_share_directory('farmbot_flatlands')
    config_file = os.path.join(pkg_share, 'config', 'simulation.yaml')

    # Load the YAML configuration file
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)

    # Extract the number of robots from the config file if not overridden by launch argument
    if 'num_robots' not in config.get('global', {}).get('ros__parameters', {}):
        config['global']['ros__parameters']['num_robots'] = num_robots

    send_goal_actions = []

    for robot_id in range(num_robots):
        # Generate the mission goal for this robot
        mission_goal = generate_mission_goal(robot_id, num_robots=num_robots)

        # Define the action server name
        action_server = f"/robot{robot_id}/nav/mission"

        # Define the action type
        action_type = "farmbot_interfaces/action/Waypoints"

        # Construct the command as a single string
        # Properly escape quotes and handle multi-line strings
        send_goal_cmd = [
            'ros2', 'action', 'send_goal',
            action_server,
            action_type,
            mission_goal
        ]
        # print(send_goal_cmd)

        # Append the ExecuteProcess action
        send_goal_actions.append(
            ExecuteProcess(
                cmd=send_goal_cmd,
                shell=False,  # Use shell=True to handle the multi-line YAML string
                # name=f"send_goal_robot{robot_id}",
                output="screen"
            )
        )

    return send_goal_actions

def generate_launch_description():
    # Declare the launch argument for number of robots
    num_robots_arg = DeclareLaunchArgument(
        'num_robots',
        default_value='1',  # Default value if not specified
        description='Number of robots to send mission goals to'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the launch argument
    ld.add_action(num_robots_arg)

    # Add an OpaqueFunction to handle the dynamic generation of send_goal commands
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
