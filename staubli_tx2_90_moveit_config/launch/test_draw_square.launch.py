#!/usr/bin/env python3
import os
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
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

    declare_initial_delay = DeclareLaunchArgument(
        "initial_delay",
        default_value="15.0",
        description="Seconds to wait inside the inline loop before first execution"
    )
    initial_delay = LaunchConfiguration("initial_delay")

    declare_period = DeclareLaunchArgument(
        "period",
        default_value="30.0",
        description="Seconds between square executions"
    )
    period = LaunchConfiguration("period")

    declare_side = DeclareLaunchArgument(
        "side",
        default_value="0.10",
        description="Side length of the square in meters"
    )
    side = LaunchConfiguration("side")

    # MoveIt config
    moveit_config = (
        MoveItConfigsBuilder("staubli_tx2_90")
        .robot_description(file_path="config/staubli_tx2_90.urdf.xacro")
        .robot_description_semantic(file_path="config/staubli_tx2_90.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .to_moveit_configs()
    )

    controller_config_path = os.path.join(pkg_moveit, "config", "staubli_tx2_90_controllers.yaml")
    controllers_yaml = xacro.load_yaml(controller_config_path)

    # RViz / TF / robot_state_publisher
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

    # parámetros de ejecución (más tolerantes para validación)
    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.execution_duration_monitoring": False,
        "trajectory_execution.allowed_execution_duration_scaling": 100.0,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.05,   # tolerancia de arranque
    }

    # evitamos que ciertas escalas queden a 0.0
    execution_scaling = {
        "max_velocity_scaling_factor": 1.0,
        "max_acceleration_scaling_factor": 1.0
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
    }

    planning_pipelines_param = {
        "planning_pipelines": {"pipeline_names": ["ompl"], "namespace": ""},
        "plan_request_params": {"planning_pipeline": "ompl", "planning_attempts": 1, "planning_time": 5.0},
        "ompl": {
            "planning_plugins": ["ompl_interface/OMPLPlanner"],
            "request_adapters": [
                "default_planning_request_adapters/ResolveConstraintFrames",
                "default_planning_request_adapters/ValidateWorkspaceBounds",
                "default_planning_request_adapters/CheckStartStateBounds",
                "default_planning_request_adapters/CheckStartStateCollision"
            ],
            "response_adapters": [
                "default_planning_response_adapters/AddTimeOptimalParameterization",
                "default_planning_response_adapters/ValidateSolution",
                "default_planning_response_adapters/DisplayMotionPath"
            ],
            "longest_valid_segment_fraction": 0.01,
            "default_planner_config": "RRTConnectkConfigDefault",
            "planner_configs": {"RRTConnectkConfigDefault": {"type": "geometric::RRTConnect", "range": 0.0}}
        }
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            trajectory_execution,
            execution_scaling,
            {"moveit_simple_controller_manager": controllers_yaml,
             "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager"},
            planning_scene_monitor_parameters,
            planning_pipelines_param
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # driver (delayed para que move_group publique parámetros primero)
    include_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_driver, "launch", "robot_interface_streaming.launch.py")),
        launch_arguments={"robot_ip": robot_ip}.items()
    )
    driver_delay_seconds = 25.0
    include_driver_delayed = TimerAction(period=driver_delay_seconds, actions=[include_driver])

    # Parámetros por defecto (si quieres, los sobreescribes con argumentos de launch)
    EE_LINK = "tool0"      # <- AJUSTA si tu link final se llama distinto
    BASE_FRAME = "base_link"
    GROUP_NAME = "manipulator"

    # SCRIPT INLINE: lee parámetros desde ENV (DRAW_PERIOD, DRAW_SIDE, DRAW_INITIAL_DELAY)
    # Implementación robusta: espera /joint_states válido, calcula cada lado por separado y ejecuta.
    draw_script = r'''
import os
import time
import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory

# Read env params
PERIOD = float(os.environ.get("DRAW_PERIOD", "30.0"))
SIDE = float(os.environ.get("DRAW_SIDE", "0.10"))
INITIAL_DELAY = float(os.environ.get("DRAW_INITIAL_DELAY", "12.0"))
EE_LINK = os.environ.get("DRAW_EE_LINK", "tool0")
BASE_FRAME = os.environ.get("DRAW_BASE_FRAME", "base_link")
GROUP_NAME = os.environ.get("DRAW_GROUP", "manipulator")

class DrawSquareNode(Node):
    def __init__(self):
        super().__init__('draw_square_inline')
        self.get_logger().info(f'draw_square node starting (side={SIDE}, period={PERIOD})')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.cart_client = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        self.exec_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        self._last_js = None
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)

    def _js_cb(self, msg):
        self._last_js = msg

    def wait_for_interfaces(self, timeout=15.0):
        if not self.cart_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error('compute_cartesian_path service NOT available after wait')
            return False
        if not self.exec_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error('execute_trajectory action NOT available after wait')
            return False
        return True

    def wait_for_valid_joint_state(self, timeout=5.0):
        t0 = time.time()
        while rclpy.ok() and (time.time()-t0) < timeout:
            if self._last_js is not None:
                try:
                    if self._last_js.header.stamp.sec != 0:
                        return True
                except Exception:
                    pass
            rclpy.spin_once(self, timeout_sec=0.05)
        return False

    def lookup_pose(self, target_frame, source_frame, timeout=5.0):
        t0 = time.time()
        while rclpy.ok() and (time.time()-t0) < timeout:
            try:
                tf = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
                pose = Pose()
                pose.position.x = tf.transform.translation.x
                pose.position.y = tf.transform.translation.y
                pose.position.z = tf.transform.translation.z
                pose.orientation = tf.transform.rotation
                return pose
            except Exception:
                rclpy.spin_once(self, timeout_sec=0.05)
        raise RuntimeError(f'Could not get transform {source_frame} -> {target_frame}')

    def compute_cartesian_segment(self, start_pose, end_pose, side_m, base_frame, ee_link, group_name):
        # Ensure we have a recent joint_state
        if not self.wait_for_valid_joint_state(timeout=3.0):
            self.get_logger().warn('No valid joint state before compute_cartesian_path; continuing but results may be poor')

        req = GetCartesianPath.Request()
        req.header = Header()
        req.header.frame_id = base_frame
        req.group_name = group_name
        req.link_name = ee_link
        req.waypoints = [start_pose, end_pose]
        # Use a max_step >= side so solver focuses on endpoints (we want planner to connect vertices)
        req.max_step = max(side_m * 1.1, 0.05)
        req.jump_threshold = 0.0
        req.avoid_collisions = True

        fut = self.cart_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        if not fut.done():
            self.get_logger().error('GetCartesianPath call timed out')
            return None, 0.0
        resp = fut.result()
        fraction = getattr(resp, 'fraction', 0.0)
        robot_traj = None
        if hasattr(resp, 'solution'):
            robot_traj = resp.solution
        elif hasattr(resp, 'trajectory'):
            robot_traj = resp.trajectory
        else:
            try:
                robot_traj = resp[0]
            except Exception:
                robot_traj = None
        return robot_traj, fraction

    def execute_robot_trajectory(self, robot_traj, timeout=120.0):
        goal_msg = ExecuteTrajectory.Goal()
        # set trajectory field safely (different versions use different names)
        if hasattr(goal_msg, 'trajectory'):
            setattr(goal_msg, 'trajectory', robot_traj)
        else:
            try:
                setattr(goal_msg, 'robot_trajectory', robot_traj)
            except Exception:
                self.get_logger().error('Cannot set trajectory on goal_msg')
                return False

        send_goal_fut = self.exec_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_fut, timeout_sec=10.0)
        if not send_goal_fut.done():
            self.get_logger().error('send_goal_async timed out')
            return False
        goal_handle = send_goal_fut.result()
        if goal_handle is None:
            self.get_logger().error('send_goal returned None')
            return False
        if not getattr(goal_handle, 'accepted', False):
            self.get_logger().error('execute_trajectory goal NOT accepted')
            return False

        res_fut = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=timeout)
        if not res_fut.done():
            self.get_logger().warn('execute_trajectory result future did not complete in time')
            return False
        res = res_fut.result()
        self.get_logger().info(f'Execution result: {res}')
        return True

    def compute_and_execute_square(self, side_m, base_frame, ee_link, group_name):
        # lookup current TCP pose in base frame
        try:
            current = self.lookup_pose(base_frame, ee_link, timeout=5.0)
        except Exception as e:
            self.get_logger().error('Failed to lookup current pose: ' + str(e))
            return False

        x0 = current.position.x
        y0 = current.position.y
        z0 = current.position.z

        corners = [
            (x0 + side_m, y0, z0),
            (x0 + side_m, y0 + side_m, z0),
            (x0,        y0 + side_m, z0),
            (x0,        y0,        z0)
        ]
        # build poses
        poses = []
        for (x,y,z) in corners:
            p = Pose()
            p.position.x = x
            p.position.y = y
            p.position.z = z
            p.orientation = current.orientation
            poses.append(p)

        # compute+execute each side separately
        for i in range(4):
            start_p = poses[i]
            end_p = poses[(i+1)%4]
            self.get_logger().info(f'Computing side {i+1} from ({start_p.position.x:.3f},{start_p.position.y:.3f}) to ({end_p.position.x:.3f},{end_p.position.y:.3f})')
            robot_traj, fraction = self.compute_cartesian_segment(start_p, end_p, side_m, base_frame, ee_link, group_name)
            self.get_logger().info(f'Fraction followed for side {i+1}: {fraction:.2f}')
            if robot_traj is None or fraction < 0.2:
                self.get_logger().warn(f'Side {i+1} fraction too small ({fraction:.2f}). Skipping execution for safety.')
                continue
            ok = self.execute_robot_trajectory(robot_traj, timeout=120.0)
            if not ok:
                self.get_logger().error(f'Execution failed for side {i+1}')
                return False
            time.sleep(0.1)
        return True

def main():
    rclpy.init()
    node = DrawSquareNode()
    if not node.wait_for_interfaces(timeout=20.0):
        node.get_logger().error('Required MoveIt interfaces not available - exiting')
        return
    node.get_logger().info(f'Initial wait {INITIAL_DELAY}s to allow planner selection in RViz')
    time.sleep(INITIAL_DELAY)
    try:
        while rclpy.ok():
            node.get_logger().info('Starting square compute+execute loop')
            ok = node.compute_and_execute_square(SIDE, BASE_FRAME, EE_LINK, GROUP_NAME)
            if not ok:
                node.get_logger().warn('Square iteration failed; will retry after period')
            node.get_logger().info(f'Sleeping {PERIOD}s before next iteration')
            time.sleep(PERIOD)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.get_logger().info('Shutting down draw node')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
'''

        # Ejecutamos el script inline (source ROS env so rclpy, moveit_msgs are available)
    draw_proc = ExecuteProcess(
        cmd=[
            "bash", "-lc",
            # crear directorios de logging y runtime, exportar variables críticas,
            # luego sourcear ROS y ejecutar el script inline
            "mkdir -p /tmp/ros_logs /tmp/runtime-root && "
            "export ROS_LOG_DIR=/tmp/ros_logs XDG_RUNTIME_DIR=/tmp/runtime-root HOME=/root && "
            "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && "
            "python3 - <<'PY'\n" + draw_script + "\nPY"
        ],
        output="screen",
        env={
            # variables que ya pasabas (LaunchConfiguration serán expandidas por launch)
            "DRAW_PERIOD": period,
            "DRAW_SIDE": side,
            "DRAW_INITIAL_DELAY": initial_delay,
            "DRAW_EE_LINK": EE_LINK,
            "DRAW_BASE_FRAME": BASE_FRAME,
            "DRAW_GROUP": GROUP_NAME,
            # garantía extra en el entorno del proceso
            "ROS_LOG_DIR": "/tmp/ros_logs",
            "XDG_RUNTIME_DIR": "/tmp/runtime-root",
            "HOME": "/root"
        }
    )


    # Start draw_proc after driver delay (allow the driver to connect and joint_states to flow)
    draw_proc_delayed = TimerAction(period=driver_delay_seconds + 3.0, actions=[draw_proc])

    ld = LaunchDescription([
        declare_robot_ip,
        declare_initial_delay,
        declare_period,
        declare_side,
        rviz_node,
        static_tf,
        robot_state_pub,
        move_group_node,
        include_driver_delayed,
        draw_proc_delayed,
    ])

    return ld
