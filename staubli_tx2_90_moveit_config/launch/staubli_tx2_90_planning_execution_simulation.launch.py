#!/usr/bin/env python3
import os
import yaml

from launch import LaunchDescription
from launch.event_handlers import OnProcessStart
from launch.actions import RegisterEventHandler, LogInfo
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    pkg = get_package_share_directory("staubli_tx2_90_moveit_config")

    #### MoveIt — Configuración del robot, cinemática y controladores
    moveit_cfg = (
        MoveItConfigsBuilder("staubli_tx2_90", package_name="staubli_tx2_90_moveit_config")
        .robot_description(file_path="config/staubli_tx2_90.urdf.xacro")
        .robot_description_semantic(file_path="config/staubli_tx2_90.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # Cargar archivo YAML de controladores
    controllers_yaml_path = os.path.join(pkg, "config", "staubli_tx2_90_controllers.yaml")
    with open(controllers_yaml_path, 'r') as f:
        controllers_dict = yaml.safe_load(f)

    # Este es el único nivel que debe pasarse al ros2_control_node como parámetros:
    ctrl_params = controllers_dict.get('controller_manager', {}).get('ros__parameters', {})

    #### Nodo robot_state_publisher (publica /robot_description necesario)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_cfg.robot_description],
        output="screen",
    )

    #### Nodo main: controller_manager / ros2_control_node
    ros2_control = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[moveit_cfg.robot_description,
                controllers_dict],  # opcional mantener update_rate aquí
    name="controller_manager",
    output="screen",
    )

    #### Spawners — inician cada controlador dentro del controller_manager
    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager", "--param-file", controllers_yaml_path],
        output="screen",
    )
    spawn_manip = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["manipulator_controller", "--controller-manager", "/controller_manager", "--param-file", controllers_yaml_path],
        output="screen",
    )

    #### Event handlers para disparar los spawners en el orden correcto
    handler_start_jsb = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control,
            on_start=[
                LogInfo(msg="[ROS2_CONTROL] ros2_control_node listo; lanzando joint_state_broadcaster"),
                spawn_jsb
            ]
        )
    )
    handler_start_manip = RegisterEventHandler(
        OnProcessStart(
            target_action=spawn_jsb,
            on_start=[
                LogInfo(msg="[SPAWNER] joint_state_broadcaster activo; lanzando manipulator_controller"),
                spawn_manip
            ]
        )
    )

    #### Movimiento de grupo MoveIt: se lanza SOLO tras activar ambos controladores
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_cfg.to_dict()],
    )
    handler_start_mg = RegisterEventHandler(
        OnProcessStart(
            target_action=spawn_manip,
            on_start=[
                LogInfo(msg="[SPAWNER] manipulator_controller activo; lanzando move_group"),
                move_group
            ]
        )
    )

    #### RViz para visualización
    rviz_config = os.path.join(pkg, "config", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_cfg.robot_description,
            moveit_cfg.robot_description_semantic,
            moveit_cfg.robot_description_kinematics,
        ],
        output="screen",
    )

    return LaunchDescription([
        robot_state_publisher,
        ros2_control,
        handler_start_jsb,
        handler_start_manip,
        handler_start_mg,
        rviz_node,
    ])
