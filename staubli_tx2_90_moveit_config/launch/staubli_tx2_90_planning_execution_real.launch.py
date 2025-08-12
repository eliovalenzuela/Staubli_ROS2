#!/usr/bin/env python3
import os
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    pkg_moveit = get_package_share_directory("staubli_tx2_90_moveit_config")
    pkg_driver = get_package_share_directory("staubli_val3_driver")

    declare_robot_ip = DeclareLaunchArgument(
        "robot_ip",
        default_value="192.168.31.99",
        description="IP address of the Staubli robot"
    )
    robot_ip = LaunchConfiguration("robot_ip")

    # MoveIt config (URDF/SRDF/kinematics/controllers)
    moveit_config = (
        MoveItConfigsBuilder("staubli_tx2_90")
        .robot_description(file_path="config/staubli_tx2_90.urdf.xacro")
        .robot_description_semantic(file_path="config/staubli_tx2_90.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    controller_config_path = os.path.join(pkg_moveit, "config", "staubli_tx2_90_controllers.yaml")
    controllers_yaml = xacro.load_yaml(controller_config_path)

    # RViz / TF / robot_state_publisher (arrancan inmediatamente)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", os.path.join(pkg_moveit, "config", "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
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

    # move_group: LO INICIAMOS ANTES que el driver (para que exponga parámetros/servicios)
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

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            trajectory_execution,
            {"moveit_simple_controller_manager": controllers_yaml,
             "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager"},
            planning_scene_monitor_parameters
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # Include del launch del driver (arrancamos con un pequeño delay para dejar
    # a move_group inicializar sus parámetros/servicios)
    include_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_driver, "launch", "robot_interface_streaming.launch.py")),
        launch_arguments={"robot_ip": robot_ip}.items()
    )

    # Ajusta este retardo si tu red o driver tardan más en inicializar (ej: 3-8s)
    driver_delay_seconds = 15.0
    include_driver_delayed = TimerAction(period=driver_delay_seconds, actions=[include_driver])

    ld = LaunchDescription([
        declare_robot_ip,
        rviz_node,
        static_tf,
        robot_state_pub,
        move_group_node,          # arrancar primero
        include_driver_delayed,   # arrancar driver con retardo
    ])

    return ld
