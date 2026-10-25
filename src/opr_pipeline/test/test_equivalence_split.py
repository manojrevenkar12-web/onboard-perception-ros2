"""The same check with the deployment split into three processes and the detector reading
rectified images in place from shared memory (zero copy)."""
import sys
import os
import unittest

import launch_testing
import pytest

sys.path.insert(0, os.path.dirname(__file__))
from equivalence_common import compare, describe  # noqa: E402


@pytest.mark.launch_test
def generate_test_description():
    return describe("clutter", 3, "split", {"detector_zero_copy": "true"})


class TestSplit(unittest.TestCase):
    def test_telemetry_matches_visionbox(self, record, golden):
        compare(self, record, golden)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_clean_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
