#!/usr/bin/env python3
import os
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
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

    # NEW: launch arg para simulación
    declare_use_sim = DeclareLaunchArgument(
        "use_simulator",
        default_value="false",
        description="true to run in simulation (no real robot), false to connect to real robot"
    )
    use_sim = LaunchConfiguration("use_simulator")

    # MoveIt config (URDF/SRDF/kinematics/controllers)
    moveit_config = (
        MoveItConfigsBuilder("staubli_tx2_90")
        .robot_description(file_path="config/staubli_tx2_90.urdf.xacro")
        .robot_description_semantic(file_path="config/staubli_tx2_90.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # paths a los yaml existentes
    controller_config_path = os.path.join(pkg_moveit, "config", "staubli_tx2_90_controllers.yaml")
    controllers_yaml = xacro.load_yaml(controller_config_path)

    # Path para fake controllers (créalo tal cual más abajo)
    fake_controller_config_path = os.path.join(pkg_moveit, "config", "fake_controllers.yaml")
    fake_controllers_yaml = {}
    if os.path.exists(fake_controller_config_path):
        fake_controllers_yaml = xacro.load_yaml(fake_controller_config_path)
    else:
        # si no existe, dejamos dict vacío; el launch fallará indicando que crees ese fichero
        fake_controllers_yaml = {}

    # nodes que siempre arrancan
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

    # publicador de estado del robot (TF desde joints)
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # Trajectory execution params (los tuyos)
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
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
    }

    # ------------------------------
    # MOVE_GROUP para modo REAL (con driver)
    move_group_real = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            trajectory_execution,
            {
                # pasamos los controllers reales y el simple controller manager
                "moveit_simple_controller_manager": controllers_yaml,
                "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
            },
            planning_scene_monitor_parameters
        ],
        arguments=["--ros-args", "--log-level", "info"],
        condition=UnlessCondition(use_sim)  # sólo si use_simulator == false
    )

    # ------------------------------
    # MOVE_GROUP para modo SIM (fake controllers)
    # usamos el plugin fake controller manager de MoveIt
    move_group_sim = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            trajectory_execution,
            {
                # pasar los parámetros para usar el fake controller manager
                "moveit_fake_controller_manager": fake_controllers_yaml,
                "moveit_controller_manager": "moveit_fake_controller_manager/MoveItFakeControllerManager",
            },
            planning_scene_monitor_parameters
        ],
        arguments=["--ros-args", "--log-level", "info"],
        condition=IfCondition(use_sim)  # sólo si use_simulator == true
    )

    # ------------------------------
    # INCLUDE driver real (lo arrancamos sólo si NO estamos en simulación)
    include_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_driver, "launch", "robot_interface_streaming.launch.py")),
        launch_arguments={"robot_ip": robot_ip}.items()
    )

    # arrancar driver con retardo sólo en modo real
    driver_delay_seconds = 20.0
    include_driver_delayed = TimerAction(
        period=driver_delay_seconds,
        actions=[include_driver],
        condition=UnlessCondition(use_sim)
    )

    # ------------------------------
    # NODOS de simulación: publicador de joint_states cuando estamos en simulación
    joint_state_pub = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
        condition=IfCondition(use_sim)
    )

    ld = LaunchDescription([
        declare_robot_ip,
        declare_use_sim,
        rviz_node,
        static_tf,
        robot_state_pub,
        move_group_real,
        move_group_sim,
        joint_state_pub,
        include_driver_delayed,   # sólo se incluirá si not use_simulator
    ])

    return ld
