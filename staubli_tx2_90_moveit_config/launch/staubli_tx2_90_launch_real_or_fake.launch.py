#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import xacro

def generate_launch_description():
    pkg_moveit = get_package_share_directory("staubli_tx2_90_moveit_config")

    # Args
    declare_robot_ip = DeclareLaunchArgument("robot_ip", default_value="192.168.31.99",
                                             description="IP del robot Staubli")
    declare_use_fake = DeclareLaunchArgument("use_fake_hardware", default_value="false",
                                             description="true -> usar FakeSystem; false -> usar driver real")

    robot_ip = LaunchConfiguration("robot_ip")
    use_fake = LaunchConfiguration("use_fake_hardware")

    # MoveIt config
    moveit_config = (
        MoveItConfigsBuilder("staubli_tx2_90")
        .robot_description(file_path="config/staubli_tx2_90.urdf.xacro")
        .robot_description_semantic(file_path="config/staubli_tx2_90.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # Path y carga del YAML de controladores (se usa tanto para fake como para MoveIt config)
    controller_config_path = os.path.join(pkg_moveit, "config", "staubli_tx2_90_controllers.yaml")
    controllers_yaml = xacro.load_yaml(controller_config_path)

    # --- Nodo ros2_control_node (solo si use_fake_hardware == true) ---
    node_ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[controllers_yaml],
        condition=IfCondition(use_fake)
    )

    # --- Spawners (solo para fake hardware) ---
    load_controllers = TimerAction(
        period=4.0,
        actions=[
            # spawner joint_state_broadcaster
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager", "--param-file", controller_config_path],
                output="screen",
                condition=IfCondition(use_fake)
            ),
            # spawner manipulator_controller
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["manipulator_controller", "--controller-manager", "/controller_manager", "--param-file", controller_config_path],
                output="screen",
                condition=IfCondition(use_fake)
            ),
        ]
    )

    # --- Include del driver Staubli (solo si use_fake_hardware == false) ---
    # Intenta incluir el launch de staubli_val3_driver/robot_interface_streaming.launch.py pasando robot_ip
    staubli_driver_launch_path = os.path.join(
        get_package_share_directory("staubli_val3_driver"),
        "launch",
        "robot_interface_streaming.launch.py"
    )
    include_staubli_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(staubli_driver_launch_path),
        launch_arguments={"robot_ip": robot_ip}.items(),
        condition=UnlessCondition(use_fake)
    )

    # --- Node move_group (siempre) configurado para no gestionar controladores (driver real expone action server) ---
    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }
    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.execution_duration_monitoring": False,
        "trajectory_execution.allowed_execution_duration_scaling": 100.0,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    node_move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), trajectory_execution, moveit_controllers, planning_scene_monitor_parameters],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # RViz, TF estático y robot_state_publisher (si quieres mostrar URDF)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", os.path.join(pkg_moveit, "config", "moveit.rviz")],
        parameters=[moveit_config.robot_description, moveit_config.robot_description_semantic, moveit_config.robot_description_kinematics],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "map", "--child-frame-id", "base_link"],
    )

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    ld = LaunchDescription()
    ld.add_action(declare_robot_ip)
    ld.add_action(declare_use_fake)
    # driver real (IncludeLaunchDescription) o ros2_control_node (condicionado)
    ld.add_action(include_staubli_driver)
    ld.add_action(node_ros2_control)
    ld.add_action(load_controllers)
    # moveit + visualization
    ld.add_action(node_move_group)
    ld.add_action(rviz_node)
    ld.add_action(static_tf)
    ld.add_action(robot_state_pub)

    return ld
