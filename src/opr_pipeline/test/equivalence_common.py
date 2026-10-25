"""Shared by the launch tests that replay a recorded flight through the ROS 2 pipeline and compare
the telemetry with visionbox 2.3's, byte for byte."""
import os
import tempfile
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions


def describe(scenario, seed, mode, extra=None):
    record = os.path.join(tempfile.mkdtemp(prefix="opr_eq_"), "telemetry.hex")
    args = {
        "mode": mode,
        # 15 Hz paced from the simulated sensor; reliable QoS end to end, so a lost frame is a
        # failure of the plumbing, not of the test's patience.
        "device": f"sim://{scenario}?seed={seed}&pace_hz=15",
        "image_qos": "reliable",
        "record_path": record,
        "udp_port": "0",
        "camera_autostart": "false",
    }
    args.update(extra or {})
    launch_file = os.path.join(get_package_share_directory("opr_pipeline"), "launch", "pipeline.launch.py")
    ld = LaunchDescription(
        [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_file), launch_arguments=args.items()),
            launch_testing.actions.ReadyToTest(),
        ]
    )
    return ld, {"record": record, "golden": os.path.join(os.environ["OPR_GOLDEN_DIR"], f"{scenario}_{seed}.hex")}


# Every topic the flight passes through, with the subscribers it needs before the camera may start.
GRAPH = {
    "/frame_info": 1,
    "/image_rect": 0,  # either raw or shm, checked below
    "/detections": 1,
    "/tracks": 2,  # bridge and health
}


def bring_up_camera(test, timeout=30.0):
    """What a lifecycle manager does: wait until everything downstream is subscribed, then
    configure and activate the camera. A camera that starts first loses its first frames to
    discovery, which no QoS setting can recover."""
    import rclpy
    from lifecycle_msgs.msg import Transition
    from lifecycle_msgs.srv import ChangeState

    rclpy.init()
    node = rclpy.create_node("lifecycle_manager")
    try:
        deadline = time.time() + timeout
        while time.time() < deadline:
            ready = all(node.count_subscribers(t) >= n for t, n in GRAPH.items())
            ready = ready and (node.count_subscribers("/image_rect") + node.count_subscribers("/image_rect/shm")) >= 1
            ready = ready and (node.count_subscribers("/image_raw") + node.count_subscribers("/image_raw/shm")) >= 1
            if ready:
                break
            rclpy.spin_once(node, timeout_sec=0.1)
        test.assertTrue(ready, "the pipeline never finished subscribing")
        client = node.create_client(ChangeState, "/camera/change_state")
        test.assertTrue(client.wait_for_service(timeout_sec=10.0), "camera lifecycle service missing")
        for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
            request = ChangeState.Request()
            request.transition.id = transition
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
            test.assertTrue(future.result() is not None and future.result().success, f"transition {transition} failed")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def compare(test, record, golden, timeout=90.0):
    bring_up_camera(test)
    with open(golden) as f:
        want = f.read().split()
    got = []
    deadline = time.time() + timeout
    while time.time() < deadline:
        if os.path.exists(record):
            with open(record) as f:
                got = f.read().split()
            if len(got) >= len(want):
                break
        time.sleep(0.5)
    test.assertEqual(len(got), len(want), f"recorded {len(got)} of {len(want)} datagrams before timeout")
    for i, (a, b) in enumerate(zip(got, want)):
        test.assertEqual(a, b, f"datagram {i} differs from visionbox 2.3")
