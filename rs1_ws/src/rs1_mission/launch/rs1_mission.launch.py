"""Launch the mission coordinator and both robot mission executors."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PKG_NAME = "rs1_mission"


def generate_launch_description():
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    log_level = LaunchConfiguration("log_level")

    launch_arguments = [
        DeclareLaunchArgument("home_from_start", default_value="true",
                              description="Save each robot's pose at mission start unless spawn poses are supplied"),
        DeclareLaunchArgument("husky_home", default_value="[0.0, 0.0, 0.0, 0.0]",
                              description="Husky spawn [x, y, z, yaw] in world"),
        DeclareLaunchArgument("parrot_home", default_value="[0.0, 0.0, 0.0, 0.0]",
                              description="Parrot spawn [x, y, z, yaw] in world"),
        DeclareLaunchArgument("home_tolerance_m", default_value="0.5"),
        DeclareLaunchArgument("tree_standoff_m", default_value="1.5"),
        DeclareLaunchArgument("tree_measurement_min_m", default_value="1.0"),
        DeclareLaunchArgument("tree_measurement_max_m", default_value="4.0"),
        DeclareLaunchArgument("tree_goal_margin_m", default_value="0.35"),
        DeclareLaunchArgument("dbh_diagnostics_dir", default_value=""),
        DeclareLaunchArgument("joystick", default_value="true"),
        DeclareLaunchArgument("joystick_device_id", default_value="0"),
        DeclareLaunchArgument("joystick_device_name", default_value=""),
        DeclareLaunchArgument("roi_frame", default_value="world"),
        DeclareLaunchArgument("session_id", default_value="manual"),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="True",
            description="Use the Gazebo simulation clock.",
        ),
        DeclareLaunchArgument(
            "roi_x",
            default_value="-40.0",
            description="ROI x coordinate in roi_frame (metres).",
        ),
        DeclareLaunchArgument(
            "roi_y",
            default_value="40.0",
            description="ROI y coordinate in roi_frame (metres).",
        ),
        DeclareLaunchArgument(
            "roi_z",
            default_value="0.0",
            description="ROI z coordinate in roi_frame (metres).",
        ),
        DeclareLaunchArgument(
            "roi_x_min",
            default_value="-45.0",
            description="ROI box minimum x in roi_frame (metres).",
        ),
        DeclareLaunchArgument(
            "roi_x_max",
            default_value="-35.0",
            description="ROI box maximum x in roi_frame (metres).",
        ),
        DeclareLaunchArgument(
            "roi_y_min",
            default_value="35.0",
            description="ROI box minimum y in roi_frame (metres).",
        ),
        DeclareLaunchArgument(
            "roi_y_max",
            default_value="45.0",
            description="ROI box maximum y in roi_frame (metres).",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="Logging level for all mission nodes.",
        ),
        DeclareLaunchArgument(
            "search_clearance",
            default_value="2.0",
            description="Clearance above the sensed surface during the sweep (m).",
        ),
        DeclareLaunchArgument(
            "transit_clearance",
            default_value="2.0",
            description="Clearance restored once the sweep finishes (m).",
        ),
        DeclareLaunchArgument(
            "swath_spacing",
            default_value="0.0",
            description="Swath spacing in metres; <= 0 selects an automatic spacing.",
        ),
        DeclareLaunchArgument(
            "waypoint_close_enough",
            default_value="3.0",
            description=(
                "Accept a sweep waypoint once the drone is this many metres away, "
                "so a physically blocked aimpoint cannot stall the lane."),
        ),
    ]

    mission_coordinator = Node(
        package=PKG_NAME,
        executable="mission_coordinator",
        name="mission_coordinator_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "home_from_start": ParameterValue(LaunchConfiguration("home_from_start"), value_type=bool),
                "husky_home": LaunchConfiguration("husky_home"),
                "parrot_home": LaunchConfiguration("parrot_home"),
                "home_tolerance_m": ParameterValue(LaunchConfiguration("home_tolerance_m"), value_type=float),
                "roi_frame": LaunchConfiguration("roi_frame"),
                "session_id": LaunchConfiguration("session_id"),
                "roi_x": ParameterValue(LaunchConfiguration("roi_x"), value_type=float),
                "roi_y": ParameterValue(LaunchConfiguration("roi_y"), value_type=float),
                "roi_z": ParameterValue(LaunchConfiguration("roi_z"), value_type=float),
                "roi_x_min": ParameterValue(LaunchConfiguration("roi_x_min"), value_type=float),
                "roi_x_max": ParameterValue(LaunchConfiguration("roi_x_max"), value_type=float),
                "roi_y_min": ParameterValue(LaunchConfiguration("roi_y_min"), value_type=float),
                "roi_y_max": ParameterValue(LaunchConfiguration("roi_y_max"), value_type=float),
                "search_clearance": ParameterValue(LaunchConfiguration("search_clearance"), value_type=float),
                "transit_clearance": ParameterValue(LaunchConfiguration("transit_clearance"), value_type=float),
                "swath_spacing": ParameterValue(LaunchConfiguration("swath_spacing"), value_type=float),
            }
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    parrot_mission_executor = Node(
        package=PKG_NAME,
        executable="parrot_mission_executor",
        namespace="parrot1",
        name="parrot_mission_executor_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "waypoint_close_enough": ParameterValue(
                    LaunchConfiguration("waypoint_close_enough"), value_type=float),
                "home_tolerance_m": ParameterValue(LaunchConfiguration("home_tolerance_m"), value_type=float),
            }
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    tree_standoff = ParameterValue(LaunchConfiguration("tree_standoff_m"), value_type=float)
    measurement_range = {
        "tree_measurement_min_m": ParameterValue(LaunchConfiguration("tree_measurement_min_m"), value_type=float),
        "tree_measurement_max_m": ParameterValue(LaunchConfiguration("tree_measurement_max_m"), value_type=float),
    }
    dbh_service = Node(
        package="rs1_perception",
        executable="measure_dbh.py",
        namespace="husky1",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, **measurement_range,
                     "diagnostics_dir": LaunchConfiguration("dbh_diagnostics_dir")}],
    )

    husky_mission_executor = Node(
        package=PKG_NAME,
        executable="husky_mission_executor",
        namespace="husky1",
        name="husky_mission_executor_node",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "tree_standoff_m": tree_standoff,
            "home_tolerance_m": ParameterValue(LaunchConfiguration("home_tolerance_m"), value_type=float),
            "tree_goal_margin_m": ParameterValue(LaunchConfiguration("tree_goal_margin_m"), value_type=float),
            **measurement_range,
            "tree_navigation_bt": get_package_share_directory(PKG_NAME)
            + "/config/tree_sampling_navigation.xml",
        }],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # Start once per mission stack, including deploy and managed dashboard launches
    carbon = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory(PKG_NAME) + "/launch/tree_carbon.launch.py"),
        launch_arguments={"use_sim_time": LaunchConfiguration("use_sim_time")}.items(),
    )

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("rs1_navigation") + "/launch/joystick.launch.py"),
        condition=IfCondition(LaunchConfiguration("joystick")),
        launch_arguments={
            "device_id": LaunchConfiguration("joystick_device_id"),
            "device_name": LaunchConfiguration("joystick_device_name"),
        }.items(),
    )

    return LaunchDescription(
        launch_arguments
        + [mission_coordinator, parrot_mission_executor, husky_mission_executor, dbh_service, carbon, joystick]
    )
