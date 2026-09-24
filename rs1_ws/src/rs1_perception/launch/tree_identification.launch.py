from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("rs1_perception")

    yolo_config = PathJoinSubstitution([package_share, "config", "yolo.yaml"])
    tree_location_config = PathJoinSubstitution(
        [package_share, "config", "tree_location.yaml"]
    )

    # Default artefact: the PyTorch weights shipped in share/models.
    default_model = PathJoinSubstitution([package_share, "models", "best.pt"])

    # Inference backend selection.  Everything else (imgsz, device, thresholds,
    # class names) is configured in config/yolo.yaml.
    backend = LaunchConfiguration("backend")
    provider = LaunchConfiguration("provider")
    model_path = LaunchConfiguration("model_path")
    onnx_model_path = LaunchConfiguration("onnx_model_path")

    declared_args = [
        DeclareLaunchArgument(
            "backend",
            default_value="auto",
            description=(
                "Inference backend: 'auto' (first usable of tensorrt -> onnx -> "
                "pytorch), 'pytorch' (best.pt), 'onnx' (best.onnx, torch-free) or "
                "'tensorrt' (best.onnx on TensorRT, NVIDIA GPU required)."
            ),
        ),
        DeclareLaunchArgument(
            "provider",
            default_value="auto",
            description=(
                "onnxruntime execution provider for the onnx/tensorrt backends: "
                "'auto', 'tensorrt', 'cuda' or 'cpu'."
            ),
        ),
        DeclareLaunchArgument(
            "model_path",
            default_value=default_model,
            description="PyTorch weights (.pt) for the pytorch backend.",
        ),
        DeclareLaunchArgument(
            "onnx_model_path",
            default_value="",
            description=(
                "ONNX model for the onnx and tensorrt backends. Empty = "
                "model_path with the extension swapped to .onnx."
            ),
        ),
    ]

    # Publishes world -> parrot1_base_link (+ the parrot static chain on /tf_static)
    # so the tree-location node can transform camera points into the global frame.
    world_broadcaster = Node(
        package="rs1_perception",
        executable="parrot_world_broadcaster.py",
        name="parrot_world_broadcaster",
        output="screen",
    )

    # YOLO tree detection, exposed through the /yolo/detect service.
    yolo_node = Node(
        package="rs1_perception",
        executable="yolo_node.py",
        name="yolo_node",
        output="screen",
        parameters=[
            yolo_config,
            {
                "backend": backend,
                "provider": provider,
                "model_path": model_path,
                "onnx_model_path": onnx_model_path,
            },
        ],
    )

    # Depth + lidar -> world-frame tree base locations, exposed through the
    # /yolo/process_tree_locations service.
    tree_location_node = Node(
        package="rs1_perception",
        executable="tree_location_processing.py",
        name="tree_location_node",
        output="screen",
        parameters=[tree_location_config],
    )

    return LaunchDescription(
        declared_args
        + [
            world_broadcaster,
            yolo_node,
            tree_location_node,
        ]
    )
