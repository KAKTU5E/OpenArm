import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable,IncludeLaunchDescription, SetLaunchConfiguration
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    pkg_arm_description = FindPackageShare('arm_description')
    pkg_arm_config = FindPackageShare('arm_config')

    xacro_path = PathJoinSubstitution([pkg_arm_description, 'urdf', 'OpenArmVKillian.xacro'])
    joint_traj_config = PathJoinSubstitution([pkg_arm_config, 'config', 'joint_traj_config.yaml'])
    
    robot_description_content = Command(['xacro ', xacro_path])

    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    robot_state_pub = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output = 'screen',
        parameters = [robot_description],
        namespace = 'controller_manager',
    )

    control_node = Node(
        package = 'controller_manager',
        executable = 'ros2_control_node',
        parameters = [joint_traj_config, robot_description],
        output = 'both', 
    )

    joint_traj_controller_spawner = Node(
        package = 'controller_manager',
        executable = 'spawner',
        arguments = ['joint_trajectory_controller', '--controller-manager', '/controller_manager']
    )

    joint_state_broadcaster_spawner = Node(
        package = 'controller_manager',
        executable = 'spawner',
        arguments = ['joint_state_broadcaster', '--controller-manager', '/controller_manager']
    )

    arm_simple_controller = Node(
        package = 'arm_config',
        executable = 'Arm_node',
        parameters=[],
        remappings=[('/something_input','/joint_states'),('/something_output','/effort_controller/commands')]
    
    )

    ld = LaunchDescription()

    ld.add_action(robot_state_pub)
    ld.add_action(control_node)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(joint_traj_controller_spawner)
    ld.add_action(arm_simple_controller)

    return ld