# Migrating visionbox 2.3 to ROS 2

visionbox is the onboard tracker this repository starts from: one C++ file, global state, capture
with `read()`, undistortion, blob detection, a Kalman tracker and a UDP telemetry stream that the
ground station parses (`src/legacy_visionbox/visionbox.cpp`, frozen). It was written for this
repository to stand for the kind of monolith a team inherits; everything else here treats it as
the real thing.

## The order of work

| Step | What changed | How we know nothing else did |
|---|---|---|
| 1. Characterise | Nothing. visionbox replays 7 simulated flight types x 3 seeds and its telemetry is kept (`src/opr_core/test/golden/`, 3,150 datagrams). | CI rebuilds visionbox and regenerates the goldens; they must equal the committed ones. |
| 2. Extract | Algorithms move out of globals into `opr_core` (Rectifier, Detector, Tracker, telemetry codec). Same arithmetic, same operation order, new structure. | `test_equivalence`: 21 flights, every datagram byte-identical. Also on arm64 (ADR 0006). |
| 3. Wrap | ROS 2 components around `opr_core`; `telemetry_bridge` speaks the old wire format so the ground station is untouched; `tools/ini_to_params.py` converts each vehicle's `visionbox.ini`. | Launch test: all nodes composed in one process reproduce the goldens byte for byte. |
| 4. Split | Detector in its own process / container, images over the shm transport, read in place. | Launch test: the three-process deployment reproduces the goldens byte for byte. |
| 5. Replace capture | `read()` loop replaced by the `opr_v4l2` mmap driver with exposure timestamps, drop counting, reconnects (findings 1, 2, 5). | Driver tests against the simulated device; the camera node's lifecycle tests. Not equivalence-tested: the old capture path is exactly what had to change. |
| 6. Retire visionbox | When a vehicle has flown the ROS 2 stack with telemetry recorded side by side (`visionbox --record` on the same camera feed is still possible), and the operators sign off. | Not done in this repository: needs flights. |

## Findings in visionbox 2.3

Found while characterising and extracting. Each is either fixed (with where) or kept on purpose.

| # | Finding | Effect | Decision |
|---|---|---|---|
| 1 | Timestamps are taken with `clock_gettime` *after* `read()` returns. | Every stamp is late by the readout time plus scheduling delay, and jitters with CPU load; the tracker's velocity estimates inherit the jitter. | Fixed in the driver: start-of-exposure buffer timestamps; end-of-frame stamps corrected by one interval; dequeue time only as a labelled fallback (`FrameInfo.stamp_source`). |
| 2 | `read()` I/O copies every frame out of the kernel and gives no sequence numbers. | Dropped frames are invisible; an extra copy per frame. | Fixed: mmap streaming; drops counted from buffer sequence numbers and published per frame (`FrameInfo.dropped_before`). |
| 3 | `dt = stamp - last` on unsigned integers. A stamp that goes backwards (clock step, reordered frame) becomes a dt of about 584 years and every track is predicted out of the image. | All tracks lost after one bad stamp. | Fixed in `opr_core::Tracker`: the frame is ignored and counted (`StepResult::accepted`, tracker diagnostics). Cannot happen on the recorded flights, so equivalence is unaffected; covered by `Track.AStampThatGoesBackwardsIsIgnored`. |
| 4 | When two targets overlap, the detector sees one merged blob, which can start and confirm a third track for a few frames. | Short-lived ghost tracks on the ground station (61 ghost track-frames across the three crossing flights; totals in `reports/tracking.md`). Target identities survive the crossing. | **Kept** for equivalence. Next change after the migration, as its own versioned behaviour change with regenerated goldens. |
| 5 | `VIDIOC_S_FMT`'s answer is not checked. A camera that does not offer 640x480 gets adjusted silently. | After a camera swap the first `read()` returns a short frame and the program exits mid-mission with `visionbox: read: Success`, instead of refusing to start. | Fixed: the driver refuses a format it did not ask for, and the camera node fails `configure`, before arming. |
| 6 | A missing `visionbox.ini` silently falls back to compiled-in defaults (a different vehicle's calibration). | A plausible-looking, wrongly calibrated vehicle. | Fixed: `ini_to_params.py` rejects missing and unknown settings; nodes log every parameter at startup. |
| 7 | Telemetry carries no notion of lost frames. | The ground station cannot tell "no targets" from "no frames". | Kept on the wire (the station's parser is not ours to change); `FrameInfo` and `perception/health` carry it on the ROS side, ready for a wire-format v2. |

## Rules while the migration is in progress

- `src/legacy_visionbox/visionbox.cpp` does not change. Its hash is pinned in `FROZEN`; changing
  it needs a line in this file saying why.
- A behaviour change in `opr_core` ships with regenerated goldens in the same commit and a row in
  the table above. Golden files are never edited by hand.
- Floating-point code keeps `-ffp-contract=off` and avoids libm transcendental functions (ADR 0006).
