# ~/staubli_tx2_90_description/launch/display.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('staubli_tx2_90_description')
    xacro_file = PathJoinSubstitution([pkg, 'urdf', 'staubli_tx2_90_description.urdf.xacro'])

    return LaunchDescription([
        # 1️⃣ Publicador de estados de articulaciones (GUI con sliders)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # 2️⃣ Publicador de estado completo del robot (generación de TF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro ', xacro_file]), value_type=str
                )
            }]
        ),

        # 3️⃣ RViz para visualizar el modelo
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
