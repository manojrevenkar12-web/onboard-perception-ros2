# 0003. Write the V4L2 capture layer instead of using a generic camera node

Status: accepted

## Context

`v4l2_camera` and similar packages are good general-purpose camera nodes. On this vehicle the
camera layer has to answer questions they leave to the user: when was this frame exposed, how many
frames did the sensor drop, what happens when the cable is pulled in flight, and does the
resolution the driver gave us match the calibration.

## Decision

`opr_v4l2` is a small memory-mapped streaming driver (QUERYCAP, S_FMT, S_PARM, REQBUFS, mmap,
QBUF/DQBUF, STREAMON/OFF) behind a syscall interface, with:

- exposure timestamps from the buffer (start of exposure; end-of-frame stamps corrected by one
  interval; dequeue time only as a labelled last resort);
- drop counting from the driver's sequence numbers;
- refusal of a silently adjusted format (visionbox never checked; a different size would have read
  past its buffers);
- RAII buffers that requeue themselves and cannot requeue into a stream they did not come from;
- stream pause/resume, so a lifecycle-inactive camera holds no stale frames;
- `FakeDevice`, a simulated camera answering the same ioctls, for tests and bench runs.

## Consequences

- More code to own. It is about 300 lines and its behaviour is tested against the fake device,
  including unplug, corrupt buffers, slow consumers and missing timestamps.
- Not tested against real hardware in this repository: there is no camera on the build machine.
  `opr_v4l2_probe` is the first thing to run on the vehicle (docs/FIELD_CHECKLIST.md).
- Next step on Jetson: `V4L2_MEMORY_USERPTR` or DMABUF into the shared-memory slots, removing the
  last copy between the sensor and every consumer.
