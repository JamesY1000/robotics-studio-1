"""Remove terrain returns before Husky's SLAM and obstacle mapping."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    tf_remaps = [("/tf", "/husky1/tf"), ("/tf_static", "/husky1/tf_static")]
    parameters = PathJoinSubstitution([
        FindPackageShare("rs1_perception"), "config", "husky_ground_segmentation.yml"])
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "ground_fallback_radius", default_value="1.5",
            description="Wider ground search when the nearby fit fails; 0.75 disables the fallback"),
        DeclareLaunchArgument(
            "ground_height_bound_enabled", default_value="false",
            description="Reject clearly overhead returns above a consistent narrow ground strip"),
        Node(
            package="rs1_perception", executable="level_husky_cloud.py",
            namespace="husky1", output="screen",
            parameters=[{"use_sim_time": use_sim_time}], remappings=tf_remaps),
        Node(
            package="ground_segmentation_ros2", executable="ground_segmentation_ros2_node",
            namespace="husky1", name="ground_segmentation", output="screen",
            parameters=[parameters, {"use_sim_time": use_sim_time}],
            remappings=tf_remaps + [
                ("/ground_segmentation/input_pointcloud", "/husky1/points_level"),
                ("/ground_segmentation/ground_points", "/husky1/ground_points"),
                ("/ground_segmentation/obstacle_points", "/husky1/obstacle_points"),
                ("/ground_segmentation/raw_points", "/husky1/segmentation_input"),
            ]),
        Node(
            package="rs1_perception", executable="ground_relative_scan_cloud.py",
            namespace="husky1", output="screen",
            parameters=[{"use_sim_time": use_sim_time,
                         "ground_fallback_radius": LaunchConfiguration("ground_fallback_radius"),
                         "ground_height_bound_enabled": ParameterValue(
                             LaunchConfiguration("ground_height_bound_enabled"), value_type=bool)}],
            remappings=tf_remaps),
        Node(
            package="pointcloud_to_laserscan", executable="pointcloud_to_laserscan_node",
            namespace="husky1", name="filtered_scan", output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "target_frame": "husky1_base_level",
                "transform_tolerance": 0.2,
                # Height above nearby ground was checked by the preceding node.
                # A narrow robot-relative band would hide trunks farther uphill.
                "min_height": -20.0, "max_height": 20.0,
                "angle_min": -3.141592653589793, "angle_max": 3.141592653589793,
                "angle_increment": 0.00436332313,
                "scan_time": 0.3333333333,
                "range_min": 0.25, "range_max": 40.0,
                # No obstacle point is NOT proof of a clear ray behind removed ground.
                # Keep empty bins non-finite; Nav2 inf_is_valid stays false.
                "use_inf": True,
            }],
            remappings=tf_remaps + [
                ("cloud_in", "/husky1/navigation_points"),
                ("scan", "/husky1/scan_filtered"),
            ]),
    ])
