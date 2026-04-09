from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default='false')
    hardware_plugin = LaunchConfiguration("hardware_plugin")
    can_interface = LaunchConfiguration("can_interface")

    pkg_arm_description = FindPackageShare('arm_description')
    pkg_arm_config = FindPackageShare('arm_config')

    xacro_path = PathJoinSubstitution([
        pkg_arm_description, 
        'urdf', 
        'AmigoArmMinimal.urdf'
    ])
    
    joint_traj_config = PathJoinSubstitution([
        pkg_arm_config, 
        'config', 
        'joint_traj_config.yaml'
    ])
    
    robot_description = {
        "robot_description": Command([
            FindExecutable(name="xacro"),
            " ",
            xacro_path,
            " ",
            "hardware_plugin:=",
            hardware_plugin,
            " ",
            "can_interface:=",
            can_interface,
        ])
    }

    hardware_plugin_arg = DeclareLaunchArgument(
        "hardware_plugin",
        default_value="cubemars_hardware/CubeMarsSystemHardware",
        description="ros2_control hardware plugin to load. Override with mock_components/GenericSystem for laptop/mock bringup.",
    )

    can_interface_arg = DeclareLaunchArgument(
        "can_interface",
        default_value="can0",
        description="CAN interface name used by the real CubeMars hardware plugin.",
    )

    robot_state_pub = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output = 'screen',
        parameters = [robot_description, {"use_sim_time": use_sim_time}],
    )

    controller_manager = Node(
        package = 'controller_manager',
        executable = 'ros2_control_node',
        parameters = [robot_description, joint_traj_config, {"use_sim_time" : use_sim_time}],
        remappings = [("~/robot_description", "/robot_description")],
        output = 'both', 
    )

    joint_traj_controller_spawner = Node(
        package = 'controller_manager',
        executable = 'spawner',
        arguments = ['arm_controller', '--controller-manager', '/controller_manager'],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package = 'controller_manager',
        executable = 'spawner',
        arguments = ['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output="screen",
    )

    arm_simple_controller = Node(
        package = 'arm_config',
        executable = 'Arm_node',
        parameters=[],
        remappings=[('',''),('','')]
    
    )

    ld = LaunchDescription()

    ld.add_action(hardware_plugin_arg)
    ld.add_action(can_interface_arg)
    ld.add_action(robot_state_pub)
    ld.add_action(controller_manager)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(joint_traj_controller_spawner)
    ld.add_action(arm_simple_controller)

    return ld
