import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder("staubli_tx2_90", package_name="staubli_tx2_90_moveit_config")
        .robot_description(file_path="config/staubli_tx2_90.urdf.xacro")
        .robot_description_semantic(file_path="config/staubli_tx2_90.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # Añadimos el nodo ros2_control_node
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            os.path.join(
                get_package_share_directory("staubli_tx2_90_moveit_config"),
                "config",
                "staubli_tx2_90_controllers.yaml"
            )
        ],
        output="screen"
    )
    return LaunchDescription([
        ros2_control_node,
    ])
