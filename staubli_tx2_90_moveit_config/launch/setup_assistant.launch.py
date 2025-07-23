from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_setup_assistant_launch
from launch.substitutions import Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

pkg_desc = get_package_share_directory('staubli_tx2_90_description')
xacro = PathJoinSubstitution([pkg_desc, 'urdf', 'staubli_tx2_90_description.urdf.xacro'])
robot_description = Command(['xacro ', xacro])

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("staubli_tx2_90", package_name="staubli_tx2_90_moveit_config").to_moveit_configs()
    return generate_setup_assistant_launch(moveit_config)
