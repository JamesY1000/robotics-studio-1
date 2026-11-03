"""Deploy a saved robot setup into an existing world, without starting Gazebo."""
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, GroupAction, IncludeLaunchDescription,
    OpaqueFunction, RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    path = Path(LaunchConfiguration("plan_file").perform(context)).resolve()
    saved = json.loads(path.read_text())
    plan, placements = saved["plan"], saved["placements"]
    world_instance_id = LaunchConfiguration("world_instance_id").perform(context)
    if not world_instance_id:
        raise ValueError("Deployment requires the running world's instance_id.")

    args = {
        "world": plan["world_id"], "session_id": plan["session_id"],
        "rviz": LaunchConfiguration("rviz").perform(context),
        "log_level": LaunchConfiguration("log_level").perform(context),
        "stack_log_level": LaunchConfiguration("stack_log_level").perform(context),
        "use_sim_time": "True", "husky": "True", "parrot": "True",
        "slam": "True", "nav2": "True", "alignment": "True",
    }
    for robot in ("husky", "parrot"):
        args.update({f"{robot}_{axis}": str(placements[robot][axis])
                     for axis in ("x", "y", "z", "yaw")})
    roi = placements.get("roi", placements["husky"])
    roi_box = placements.get("roi_box")
    roi_args = {"roi_x": str(roi["x"]), "roi_y": str(roi["y"]), "roi_z": str(roi["z"]),
                "roi_frame": "world", "session_id": plan["session_id"],
                "log_level": args["log_level"]}
    if roi_box is not None:
        roi_args.update({f"roi_{axis}": str(roi_box[axis]) for axis in ("x_min", "x_max", "y_min", "y_max")})

    def include(package, file, arguments):
        return GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(Path(get_package_share_directory(package)) / "launch" / file)
            ), launch_arguments=arguments.items(),
        )])

    guard = Node(
        package="rs1_sim", executable="simulation_session.py",
        name="deployment_world_guard", output="screen",
        parameters=[{
            "use_sim_time": False, "mode": "guard", "world_instance_id": world_instance_id,
        }],
    )
    return [
        RegisterEventHandler(OnProcessExit(
            target_action=guard,
            on_exit=[EmitEvent(event=Shutdown(reason="Deployment world guard exited."))],
        )),
        guard,
        include("rs1_sim", "robot_bringup.launch.py", args),
        include("rs1_mission", "rs1_mission.launch.py", roi_args),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("plan_file"),
        DeclareLaunchArgument("world_instance_id"),
        DeclareLaunchArgument("rviz", default_value="False"),
        DeclareLaunchArgument("log_level", default_value="info", choices=["debug", "info", "warn", "error", "fatal"]),
        DeclareLaunchArgument("stack_log_level", default_value="warn", choices=["debug", "info", "warn", "error", "fatal"]),
        OpaqueFunction(function=start),
    ])
