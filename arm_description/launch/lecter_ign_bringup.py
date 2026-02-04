#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# ROS 2 launch file for bringing up a URDF robot in Gazebo Fortress (Ignition)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    # ----------------------
    # Launch Arguments
    # ----------------------
    urdf_file_arg = DeclareLaunchArgument(
        'urdf',
        default_value='/home/openmutt/openarm_ws/src/arm_description/LECTER_Full_Assembly/urdf/LECTER_BIT_BY_BIT.urdf',
        description='URDF or Xacro file to load into Gazebo Fortress.'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='Gazebo Fortress world file.'
    )

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='LECTER_Full_Assembly',
        description='Name of the robot to spawn.'
    )

    # ----------------------
    # Configurations
    # ----------------------
    urdf = LaunchConfiguration('urdf')
    world = LaunchConfiguration('world')
    robot_name = LaunchConfiguration('robot_name')

    # ----------------------
    # Nodes / Processes
    # ----------------------

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro ', urdf])}]
    )

    # Start Gazebo Fortress (ignition gazebo)
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', world, '-v', '4'],
        output='screen'
    )

    # Spawn robot into Gazebo using ros_ign_gazebo create
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', robot_name,
                   '-topic', 'robot_description'],
        output='screen'
    )

    # ----------------------
    # Launch Description
    # ----------------------
    return LaunchDescription([
        urdf_file_arg,
        world_arg,
        robot_name_arg,
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])
