# 0005. Lifecycle camera, composition where nodes share a computer, managed bring-up order

Status: accepted

## Decision

- The camera is a lifecycle node: configure opens and validates the device (a wrong or missing
  camera fails before arming), activate starts streaming, deactivate pauses it without losing the
  buffers. While active it reconnects by itself with backoff; health says ERROR until frames flow.
- Nodes that run on the same computer and are released together are components in one container
  with intra-process communication (`mode:=composed`). A module released separately runs in its own
  process or container and gets images over the shm transport (`mode:=split`).
- The camera is activated last, after everything downstream has subscribed (`camera_autostart:=false`
  plus a lifecycle manager; the equivalence tests do exactly this). A camera that starts first loses
  its first frames to discovery, and no QoS setting brings them back: the split equivalence test
  lost its first datagram until this was done.
- Every node publishes diagnostics; `perception_health` reduces them, plus "no tracks for 500 ms",
  to one level the autonomy stack can act on.
