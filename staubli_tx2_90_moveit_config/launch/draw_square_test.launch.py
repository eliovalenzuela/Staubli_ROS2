#!/usr/bin/env python3
import os
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess   # <- añade ExecuteProcess
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
        # nos aseguramos de que MoveIt publique URDF/SRDF para clientes externos
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            trajectory_execution,
            {
                "moveit_simple_controller_manager": controllers_yaml,
                "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
            },
            planning_scene_monitor_parameters,
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    include_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_driver, "launch", "robot_interface_streaming.launch.py")),
        launch_arguments={"robot_ip": robot_ip}.items(),
    )
    include_driver_delayed = TimerAction(period=20.0, actions=[include_driver])

    # --- NUEVO: ejecutar tu script draw_square.py 30 s después de lanzar move_group ---
    draw_square_proc = TimerAction(
        period=30.0,
        actions=[
            ExecuteProcess(
                cmd=["/usr/bin/env", "python3", "/ros2_ws/scripts/draw_square.py"],
                output="screen",
                # Hereda el entorno ya "sourced" desde tu terminal/launch
            )
        ],
    )

 # === NUEVO: ruta al params-file para MoveItPy ===
    draw_params_file = os.path.join(pkg_moveit, "config", "draw_square_params.yaml")

    # === NUEVO: lanzar tu script con esos parámetros ===
    # Si tu script está en /ros2_ws/scripts/draw_square.py, usa esa ruta absoluta:
    draw_script_path = "/ros2_ws/scripts/draw_square.py"

    draw_proc = ExecuteProcess(
    cmd=["/usr/bin/env", "python3", "/ros2_ws/scripts/draw_square.py"],
    output="screen",
    env=os.environ,
    )

    draw_delayed = TimerAction(period=30.0, actions=[draw_proc])

    return LaunchDescription([
        declare_robot_ip,
        rviz_node,
        static_tf,
        robot_state_pub,
        move_group_node,
        include_driver_delayed,
        draw_square_proc,     # <-- aquí
        draw_delayed
    ])
