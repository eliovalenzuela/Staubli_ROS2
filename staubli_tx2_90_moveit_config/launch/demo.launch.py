# ~/ros2_ws/src/Staubli_ROS2/staubli_tx2_90_moveit_config/launch/demo.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 🔹 Paths de paquetes
    pkg_desc = get_package_share_directory('staubli_tx2_90_description')
    pkg_moveit = get_package_share_directory('staubli_tx2_90_moveit_config')

    # 🔹 Archivo XACRO
    xacro_file = PathJoinSubstitution([pkg_desc, 'urdf', 'staubli_tx2_90_description.urdf.xacro'])

    # 🔹 Robot description (URDF generado)
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str
    )

    # 🔹 ros2_control config
    ros2_controllers = PathJoinSubstitution([pkg_moveit, 'config', 'ros2_controllers.yaml'])

    # 🔹 1. Nodo que inicia ros2_control con tus controladores
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='ros2_control_node',
        parameters=[ros2_controllers],
        output='screen'
    )

    # 🔹 2. Spawners (inicializadores) de controladores
    joint_state_pub = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    manipulator_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['manipulator_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # 🔹 3. Configuración de MoveIt
    moveit_config = (
        MoveItConfigsBuilder("staubli_tx2_90", package_name="staubli_tx2_90_moveit_config")
        .robot_description()
        .robot_description_semantic()
        .robot_description_kinematics()
        .trajectory_execution(file_path="config/ros2_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # 🔹 4. Nodo principal de Move Group
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            {'ros_control_namespace': '/controller_manager'},
            {'moveit_controller_manager': 'moveit_ros_control_interface::Ros2ControlManager'},
            {'use_sim_time': False}
        ]
    )

    # 🔹 5. RViz con configuración de MoveIt
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=[
            '-d', f"{pkg_moveit}/rviz/moveit.rviz"
        ],
        parameters=[
            {'robot_description': robot_description},
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines
        ]
    )

    # ✅ Lanzamiento ordenado
    return LaunchDescription([
        ros2_control_node,
        joint_state_pub,
        manipulator_spawner,
        move_group_node,
        rviz_node
    ])
