"""Repository-level checks that keep the migration's paperwork honest."""
import pathlib
import re
import subprocess
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import ini_to_params  # noqa: E402

INI = ROOT / "src/legacy_visionbox/visionbox.ini"
YAML = ROOT / "src/opr_pipeline/config/pipeline.yaml"


def test_committed_parameters_are_the_converted_field_config():
    assert ini_to_params.convert(INI.read_text()) == YAML.read_text()


def test_unknown_setting_is_an_error_not_a_silent_drop():
    with pytest.raises(ValueError, match="exposure_us"):
        ini_to_params.convert(INI.read_text() + "\n[camera2]\nexposure_us = 800\n")


def test_missing_setting_is_an_error():
    text = INI.read_text().replace("k3 = -0.012\n", "")
    with pytest.raises(ValueError, match="k3"):
        ini_to_params.convert(text)


def test_cli_reports_errors_with_exit_code(tmp_path):
    bad = tmp_path / "bad.ini"
    bad.write_text("[camera]\nwidth = 640\n")
    r = subprocess.run([sys.executable, str(ROOT / "tools/ini_to_params.py"), str(bad)], capture_output=True, text=True)
    assert r.returncode == 1
    assert "missing settings" in r.stderr


def test_core_defaults_match_the_field_config():
    """opr_core's defaults are what a node uses when a parameter is missing: they must be the field
    values, not whatever someone typed while refactoring."""
    header = (ROOT / "src/opr_core/include/opr_core/params.hpp").read_text()
    defaults = dict(re.findall(r"\b(?:int|double)\s+(\w+)\s*=\s*([-0-9.]+);", header))
    params = dict(re.findall(r"^\s{4}(\w+):\s*([-0-9.]+)$", YAML.read_text(), re.M))
    assert set(defaults) == set(params), set(defaults) ^ set(params)
    for key, value in params.items():
        assert float(defaults[key]) == float(value), key


def test_legacy_program_is_frozen():
    """visionbox.cpp is the reference the equivalence tests replay. Changing it would move the
    goalposts; its hash is pinned here and changes only with a written reason in MIGRATION.md."""
    import hashlib

    digest = hashlib.sha256((ROOT / "src/legacy_visionbox/visionbox.cpp").read_bytes()).hexdigest()
    pinned = (ROOT / "src/legacy_visionbox/FROZEN").read_text().split()[0]
    assert digest == pinned
