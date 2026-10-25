"""Onboard perception chain.

mode:=composed  one process, every node in one container with intra-process communication:
                images move between nodes as pointers, never serialised.
mode:=split     how a vehicle with separately released modules runs it: camera and rectification in
                one container, the detector in its own process (as a separately released or
                third-party module would be, typically in its own Docker container), tracking and
                telemetry in a third. Images cross process boundaries through the shm transport;
                detector_zero_copy:=true reads them in place.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

PKG = "opr_pipeline"
PLUGIN = "opr::pipeline::"


def _setup(context):
    get = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    params = os.path.join(get_package_share_directory(PKG), "config", "pipeline.yaml")
    mode = get("mode")
    qos = get("image_qos")
    camera = {
        "device": get("device"),
        "autostart": get("camera_autostart").lower() == "true",
        "image_qos": qos,
        "fps": 30,
    }
    bridge = {
        "udp_host": get("udp_host"),
        "udp_port": int(get("udp_port")),
        "record_path": get("record_path"),
    }
    common = {"image_qos": qos}
    zero_copy = get("detector_zero_copy").lower() == "true"
    detector_transport = "raw" if mode == "composed" else get("detector_transport")

    def comp(plugin, name, extra=None):
        return ComposableNode(
            package=PKG,
            plugin=PLUGIN + plugin,
            name=name,
            parameters=[params, common, extra or {}],
            extra_arguments=[{"use_intra_process_comms": True}],
        )

    if mode == "composed":
        return [
            ComposableNodeContainer(
                name="perception",
                namespace="",
                package="rclcpp_components",
                executable="component_container_mt",
                composable_node_descriptions=[
                    comp("CameraNode", "camera", camera),
                    comp("RectifyNode", "rectify"),
                    comp("DetectorNode", "detector", {"transport": "raw"}),
                    comp("TrackerNode", "tracker"),
                    comp("TelemetryBridgeNode", "telemetry_bridge", bridge),
                    comp("HealthNode", "perception_health"),
                ],
                output="screen",
            )
        ]
    if mode != "split":
        raise RuntimeError("mode must be composed or split, got " + mode)
    groups = {
        "camera": ComposableNodeContainer(
            name="camera_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                comp("CameraNode", "camera", camera),
                comp("RectifyNode", "rectify"),
            ],
            output="screen",
        ),
        "detector": Node(
            package=PKG,
            executable="detector_node",
            name="detector",
            parameters=[params, common, {"transport": detector_transport, "zero_copy": zero_copy}],
            output="screen",
        ),
        "tracking": ComposableNodeContainer(
            name="tracking_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                comp("TrackerNode", "tracker"),
                comp("TelemetryBridgeNode", "telemetry_bridge", bridge),
                comp("HealthNode", "perception_health"),
            ],
            output="screen",
        ),
    }
    role = get("role")
    if role == "all":
        return list(groups.values())
    if role not in groups:
        raise RuntimeError("role must be all, camera, detector or tracking, got " + role)
    return [groups[role]]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="composed", description="composed | split"),
            DeclareLaunchArgument(
                "camera_autostart",
                default_value="true",
                description="false: leave the camera unconfigured for a lifecycle manager to bring up "
                "once everything downstream is subscribed",
            ),
            DeclareLaunchArgument(
                "device",
                default_value="sim://crossing?pace_hz=30&loop=1&stamps=clock",
                description="/dev/videoN, or sim://<scenario>?... for the simulated camera",
            ),
            DeclareLaunchArgument("image_qos", default_value="sensor", description="sensor | reliable"),
            DeclareLaunchArgument(
                "role",
                default_value="all",
                description="split mode: all, or one of camera | detector | tracking (one per container)",
            ),
            DeclareLaunchArgument("detector_transport", default_value="shm", description="split mode: raw | shm"),
            DeclareLaunchArgument("detector_zero_copy", default_value="false"),
            DeclareLaunchArgument("udp_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("udp_port", default_value="14560"),
            DeclareLaunchArgument("record_path", default_value=""),
            OpaqueFunction(function=_setup),
        ]
    )
