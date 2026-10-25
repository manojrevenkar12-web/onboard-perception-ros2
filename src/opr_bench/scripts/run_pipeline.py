#!/usr/bin/env python3
"""Runs the whole perception chain from the simulated 30 Hz camera to the telemetry bridge, in each
deployment, with and without a one-core CPU limit, and measures:

  delivered   frames that produced a TrackArray / frames the camera delivered
  latency     TrackArray received by a probe minus the frame's exposure stamp (CLOCK_MONOTONIC)
  CPU         total CPU time of every pipeline process per delivered frame

The probe (an rclpy node in this script) is not pinned: it is the observer, not the workload.
Writes reports/benchmarks/pipeline.json and .md.
Usage: run_pipeline.py <repo>/reports/benchmarks [--seconds 20]
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import time

import rclpy
from opr_msgs.msg import FrameInfo, TrackArray

DEPLOYMENTS = [
    ("composed", "one process, intra-process", ["mode:=composed"]),
    ("split_raw", "3 processes, raw images", ["mode:=split", "detector_transport:=raw"]),
    ("split_shm", "3 processes, shm + copy", ["mode:=split", "detector_transport:=shm"]),
    ("split_zero_copy", "3 processes, shm in place", ["mode:=split", "detector_zero_copy:=true"]),
]
LIMITS = [("none", []), ("one core", ["taskset", "-c", "0"])]
DEVICE = "sim://swarm?pace_hz=30&loop=1&stamps=clock"


def descendants(pid):
    out, frontier = [], [pid]
    while frontier:
        p = frontier.pop()
        try:
            with open(f"/proc/{p}/task/{p}/children") as f:
                kids = [int(k) for k in f.read().split()]
        except OSError:
            kids = []
        out.extend(kids)
        frontier.extend(kids)
    return out


def cpu_seconds(pid):
    try:
        with open(f"/proc/{pid}/stat") as f:
            fields = f.read().rsplit(")", 1)[1].split()
        tick = os.sysconf("SC_CLK_TCK")
        return (int(fields[11]) + int(fields[12])) / tick  # utime + stime
    except OSError:
        return 0.0


def run(extra, prefix, seconds):
    env = dict(os.environ, RCUTILS_LOGGING_SEVERITY_THRESHOLD="WARN")
    cmd = prefix + ["ros2", "launch", "opr_pipeline", "pipeline.launch.py", f"device:={DEVICE}", "udp_port:=0"] + extra
    proc = subprocess.Popen(cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    node = rclpy.create_node("pipeline_probe")
    infos, tracks = {}, []

    def on_info(m):
        infos[(m.header.stamp.sec, m.header.stamp.nanosec)] = m.sequence

    def on_tracks(m):
        now = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
        stamp = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec
        tracks.append(((m.header.stamp.sec, m.header.stamp.nanosec), (now - stamp) / 1e6))

    node.create_subscription(FrameInfo, "/frame_info", on_info, 300)
    node.create_subscription(TrackArray, "/tracks", on_tracks, 300)
    warmup_end = time.time() + 6.0
    while time.time() < warmup_end:
        rclpy.spin_once(node, timeout_sec=0.05)
    infos.clear()
    tracks.clear()
    pids = [proc.pid] + descendants(proc.pid)
    cpu0 = {p: cpu_seconds(p) for p in pids}
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    cpu = sum(cpu_seconds(p) - cpu0.get(p, 0.0) for p in pids)
    os.killpg(proc.pid, signal.SIGINT)
    try:
        proc.wait(timeout=20)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
    node.destroy_node()
    # Count only frames whose FrameInfo arrived inside the window, so the edges of the window
    # cannot make "tracked" exceed "delivered".
    in_window = [(k, l) for k, l in tracks if k in infos]
    lat = sorted(l for _, l in in_window)
    seqs = sorted(infos.values())
    camera_frames = (seqs[-1] - seqs[0] + 1) if seqs else 0
    tracks = in_window
    pick = lambda q: lat[min(len(lat) - 1, int(q * (len(lat) - 1) + 0.5))] if lat else 0.0  # noqa: E731
    return {
        "camera_frames": camera_frames,
        "frame_info_received": len(seqs),
        "sensor_frames_lost": camera_frames - len(seqs),
        "tracked_frames": len(tracks),
        "latency_ms_p50": pick(0.5),
        "latency_ms_p99": pick(0.99),
        "latency_ms_max": pick(1.0),
        "cpu_s": cpu,
        "cpu_ms_per_frame": 1000.0 * cpu / max(len(tracks), 1),
        "processes": len(pids),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out_dir")
    ap.add_argument("--seconds", type=float, default=20.0)
    args = ap.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)
    rclpy.init()
    results = []
    for limit_name, prefix in LIMITS:
        for key, name, extra in DEPLOYMENTS:
            r = run(extra, prefix, args.seconds)
            r.update({"deployment": key, "description": name, "limit": limit_name})
            results.append(r)
            print(f"{limit_name:9} {name:28} tracked {r['tracked_frames']}/{r['camera_frames']} "
                  f"p50 {r['latency_ms_p50']:.2f} ms p99 {r['latency_ms_p99']:.2f} ms "
                  f"cpu {r['cpu_ms_per_frame']:.2f} ms/frame", flush=True)
    rclpy.shutdown()
    info = {
        "cpus": os.cpu_count(),
        "cpu_model": next((l.split(":", 1)[1].strip() for l in open("/proc/cpuinfo") if l.startswith("model name")), "?"),
        "seconds_per_run": args.seconds,
        "device": DEVICE,
    }
    with open(os.path.join(args.out_dir, "pipeline.json"), "w") as f:
        json.dump({"machine": info, "results": results}, f, indent=2)
    lines = [
        "# The whole chain: camera to telemetry",
        "",
        f"Machine: {info['cpus']} vCPU, {info['cpu_model']}. Simulated 640x480 camera at 30 Hz (swarm, 8 targets), "
        f"{args.seconds:.0f} s per run after 6 s warm-up. Generated by `opr_bench/scripts/run_pipeline.py`. "
        "CPU includes the simulated camera rendering each frame (about 3 ms) and the `ros2 launch` process.",
        "",
        "| CPU limit | Deployment | Frames tracked | Exposure to tracks p50 | p99 | CPU per frame (all processes) |",
        "|---|---|---:|---:|---:|---:|",
    ]
    for r in results:
        lines.append(f"| {r['limit']} | {r['description']} | {r['tracked_frames']}/{r['camera_frames']} | "
                     f"{r['latency_ms_p50']:.2f} ms | {r['latency_ms_p99']:.2f} ms | {r['cpu_ms_per_frame']:.2f} ms |")
    with open(os.path.join(args.out_dir, "pipeline.md"), "w") as f:
        f.write("\n".join(lines) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
