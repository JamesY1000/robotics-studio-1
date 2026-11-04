"""Launch one saved, validated dashboard session. No browser-owned processes."""
import json
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def start(context):
    path = Path(LaunchConfiguration("plan_file").perform(context)).resolve()
    saved = json.loads(path.read_text())
    plan, placements = saved["plan"], saved["placements"]
    args = {"world": plan["world_id"], "world_file": str(path.parent / "world.sdf"),
            "headless": LaunchConfiguration("headless").perform(context), "rviz": "False",
            "slam": "True", "nav2": "True", "alignment": "True",
            "session_id": plan["session_id"]}
    for robot in ("husky", "parrot"):
        args.update({f"{robot}_{axis}": str(value) for axis, value in placements[robot].items()})
    def include(package, file, arguments):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory(package)) / "launch" / file)),
            launch_arguments=arguments.items())
    roi = placements["roi"]
    roi_box = placements.get("roi_box")
    roi_args = {"roi_x": str(roi["x"]), "roi_y": str(roi["y"]),
                "roi_z": str(roi["z"]), "roi_frame": "world", "session_id": plan["session_id"]}
    if roi_box is not None:
        roi_args.update({f"roi_{axis}": str(roi_box[axis]) for axis in ("x_min", "x_max", "y_min", "y_max")})
    return [
        include("rs1_sim", "rs1_sim.launch.py", args),
        include("rs1_mission", "rs1_mission.launch.py", roi_args),
    ]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument("plan_file"),
                              DeclareLaunchArgument("headless", default_value="True"),
                              OpaqueFunction(function=start)])
