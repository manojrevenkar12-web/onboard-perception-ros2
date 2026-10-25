#!/usr/bin/env python3
"""Converts a visionbox.ini field configuration into the ROS 2 parameter file the pipeline loads.

Every vehicle in the field has its own visionbox.ini (its own lens calibration). Migrating a
vehicle is: run this on its ini, deploy the yaml. Unknown keys are an error, not a silent drop: a
setting that does not carry over must be decided by a person.

Usage: ini_to_params.py visionbox.ini > pipeline.yaml
"""
import configparser
import sys

# ini key -> (ROS parameter, type). Same names on purpose: grep finds both sides.
KEYS = {
    "width": int, "height": int,
    "fx": float, "fy": float, "cx": float, "cy": float,
    "k1": float, "k2": float, "p1": float, "p2": float, "k3": float,
    "bg_radius": int, "threshold": int, "min_area": int, "max_area": int,
    "gate": float, "accel_noise": float, "meas_noise": float,
    "init_pos_var": float, "init_vel_var": float,
    "confirm_hits": int, "max_misses": int,
}
HEADER = (
    "# Generated from src/legacy_visionbox/visionbox.ini by tools/ini_to_params.py; do not edit by hand.\n"
    "# tests/test_tools.py fails if the two disagree.\n"
)


def convert(text):
    parser = configparser.ConfigParser(inline_comment_prefixes=(";", "#"))
    parser.read_string(text)
    values = {}
    for section in parser.sections():
        for key, raw in parser.items(section):
            if key not in KEYS:
                raise ValueError(f"[{section}] {key}: no ROS 2 parameter for this setting; decide what it becomes")
            if key in values:
                raise ValueError(f"{key} is set twice")
            values[key] = KEYS[key](raw)
    missing = [k for k in KEYS if k not in values]
    if missing:
        raise ValueError("missing settings: " + ", ".join(missing))
    lines = [HEADER.rstrip("\n"), "/**:", "  ros__parameters:"]
    for key in KEYS:  # stable order
        v = values[key]
        lines.append(f"    {key}: {v!r}" if isinstance(v, float) else f"    {key}: {v}")
    return "\n".join(lines) + "\n"


def main():
    if len(sys.argv) != 2:
        print(__doc__, file=sys.stderr)
        return 2
    try:
        with open(sys.argv[1]) as f:
            sys.stdout.write(convert(f.read()))
    except ValueError as e:
        print(f"ini_to_params: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
