# 0002. Perception algorithms live in a plain C++ library; nodes are thin

Status: accepted

## Context

The monolith mixed capture, algorithms, tracking state and networking in globals. ROS 2 code
mixed with algorithms is just as hard to test and ties the algorithms to one middleware version.

## Decision

`opr_core` (rectify, detect, track, telemetry) and `opr_sim` are plain CMake packages with no ROS
dependency. Each ROS 2 node in `opr_pipeline` converts messages to `opr_core` views and back, and
owns parameters, QoS and diagnostics. `opr_core::Pipeline` composes the whole chain in one object
for the equivalence tests and as a single-process fallback.

## Consequences

- The algorithms' tests run in seconds without a ROS installation, on any architecture; CI runs
  them on emulated arm64 to check the Jetson gets the same bits.
- Image data crosses into `opr_core` as non-owning views with a row step, so the same code reads a
  ROS message, a V4L2 buffer or a shared-memory slot without copying.
