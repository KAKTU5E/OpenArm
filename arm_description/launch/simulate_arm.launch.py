import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    arm_description_path = FindPackageShare('arm_description')
    default_model_path = PathJoinSubstitution(['urdf', 'OpenArm.urdf'])
    #default_model_path = PathJoinSubstitution(['LECTER_Full_Assembly', 'urdf', 'LECTER_Full_Assembly_TORNUP.urdf'])
    default_rviz_config_path = PathJoinSubstitution([arm_description_path, 'rviz', 'urdf.rviz'])
    
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='absolute path to robot URDF file'
    )

    rviz_arg = DeclareLaunchArgument(
        name='rviz',
        default_value=default_rviz_config_path,
        description='absolute path to rviz config file'

    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
           'robot_description': Command(['xacro ', PathJoinSubstitution([arm_description_path, 'urdf', 'OpenArm.urdf'])]),
           #'robot_description': Command(['xacro ', PathJoinSubstitution([arm_description_path, 'LECTER_Full_Assembly', 'urdf', 'LECTER_Full_Assembly_TORNUP.urdf'])])
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz')]
    )



    return LaunchDescription([
        model_arg,
        rviz_arg,
        robot_state_publisher_node,
        rviz_node,

    ])