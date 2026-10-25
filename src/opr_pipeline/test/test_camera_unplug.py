"""Field drill: the camera cable comes out for a second mid-flight. Health must go to ERROR while
it is out and back to OK afterwards, without anyone restarting anything, and tracks must resume."""
import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from opr_msgs.msg import Health, TrackArray


@pytest.mark.launch_test
def generate_test_description():
    launch_file = os.path.join(get_package_share_directory("opr_pipeline"), "launch", "pipeline.launch.py")
    args = {
        "mode": "composed",
        "device": "sim://swarm?pace_hz=30&loop=1&stamps=clock&unplug_at=90&unplug_ms=1000",
        "udp_port": "0",
    }
    return LaunchDescription(
        [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_file), launch_arguments=args.items()),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestUnplug(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("unplug_drill")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_health_follows_the_camera(self):
        levels = []  # (time, level, worst)
        tracks = []
        self.node.create_subscription(Health, "/perception/health", lambda m: levels.append((time.time(), m.level, m.worst)), 10)
        self.node.create_subscription(TrackArray, "/tracks", lambda m: tracks.append((time.time(), len(m.tracks))), 50)
        end = time.time() + 20
        saw_ok_before = saw_error = saw_ok_after = False
        while time.time() < end and not saw_ok_after:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if levels:
                _, level, _ = levels[-1]
                if level == Health.OK and not saw_error:
                    saw_ok_before = True
                elif level == Health.ERROR and saw_ok_before:
                    saw_error = True
                elif level == Health.OK and saw_error:
                    saw_ok_after = True
        self.assertTrue(saw_ok_before, "health never reported OK before the unplug")
        self.assertTrue(saw_error, f"health never reported ERROR during the unplug: {levels[-5:]}")
        self.assertTrue(saw_ok_after, "health did not recover after the camera came back")
        error_times = [t for t, level, _ in levels if level == Health.ERROR]
        resumed = [t for t, n in tracks if t > max(error_times) and n > 0]
        self.assertTrue(resumed, "no tracks after recovery")
