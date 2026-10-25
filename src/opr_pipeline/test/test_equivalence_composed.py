"""A recorded flight through every node in one container (intra-process): the telemetry must be
visionbox 2.3's, byte for byte."""
import sys
import os
import unittest

import launch_testing
import pytest

sys.path.insert(0, os.path.dirname(__file__))
from equivalence_common import compare, describe  # noqa: E402


@pytest.mark.launch_test
def generate_test_description():
    return describe("occlusion", 2, "composed")


class TestComposed(unittest.TestCase):
    def test_telemetry_matches_visionbox(self, record, golden):
        compare(self, record, golden)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_clean_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
