# 0004. Leased shared-memory image slots, with automatic fallback to raw

Status: accepted

## Context

Images must reach a detector that runs in its own container (a separately released or third-party
module). Measured on this machine (reports/benchmarks/transport.md), 1080p YUYV frames through the
default middleware arrive late and, with best-effort QoS, mostly not at all: 4 MB samples exceed
the socket buffers. Options considered:

| Option | Why not (here) |
|---|---|
| Loaned messages (iceoryx, Fast DDS data sharing) | need bounded, fixed-size message types; `sensor_msgs/Image` is unbounded, so every consumer would change type |
| rmw_zenoh shared memory | promising; changes the middleware for the whole vehicle, and container setups still need shared IPC. Worth evaluating as a separate decision |
| Fast DDS LARGE_DATA / tuned buffers | measured in the benchmark; still serialises and copies each frame per subscriber |
| NITROS / type adaptation | NVIDIA-specific GPU path; the legacy code and this detector are CPU |

## Decision

`opr_shm`: a POSIX shared-memory segment per image topic with N slots. A writer fills a free slot
and publishes a small handle (`opr_msgs/ShmImage`) on `<topic>/shm`. A reader leases the slot
(CLOCK_MONOTONIC deadline), reads in place, and validates the slot's sequence afterwards; the writer
never reuses a leased slot, and a crashed reader's lease simply expires. Readers that cannot open
the segment (another host, or a container without shared IPC) fall back to the raw topic by
themselves and say so in their diagnostics.

It is exposed three ways: an image_transport plugin (`shm`) for any ROS 2 node, a dual publisher
that only copies for audiences that exist, and an in-place reader for the detector.

## Consequences

- Same topic names and message type for everyone else: rviz, ros2 bag and `raw` subscribers keep
  working.
- Cross-container use needs a shared IPC namespace and /dev/shm (`ipc: service:camera` in
  docker/compose.yaml). Without it the system still works, at raw-transport cost.
- A slow reader holds at most its lease; if every slot is leased the writer drops the frame for
  shared-memory readers and counts it, rather than block the camera.
- Correctness rests on the lease protocol; `test_segment` runs a two-process stress test that
  provokes reuse mid-read and checks that no inconsistent frame is ever accepted.
