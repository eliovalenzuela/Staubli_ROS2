from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command

def generate_launch_description():
    pkg_share = get_package_share_directory('staubli_tx2_90_description')
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'staubli_tx2_90_support.urdf.xacro'])

    resp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file])
        }]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([pkg_share, 'rviz', 'display.rviz'])]
    )

    return LaunchDescription([resp, rviz])