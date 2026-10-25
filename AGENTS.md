# Working on onboard-perception-ros2

## Fast feedback

```bash
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --packages-select opr_sim opr_core opr_v4l2 opr_shm   # seconds, no ROS needed by these
colcon test                                                     # everything, incl. launch drills (~1 min)
colcon test-result --verbose
python3 -m pytest -q tests/                                     # repository checks
```

## Invariants (each has a test)

- **The ground station sees the same bytes.** `opr_core` and every deployment of the pipeline
  reproduce visionbox 2.3's telemetry on all recorded flights (`test_equivalence`,
  `test_equivalence_composed.py`, `test_equivalence_split.py`). A deliberate behaviour change
  regenerates the goldens from code in the same commit and adds a row to docs/MIGRATION.md.
- **visionbox.cpp is frozen** (`src/legacy_visionbox/FROZEN`). CI regenerates the goldens from it.
- **Same bits on x86 and aarch64**: `-ffp-contract=off`, no libm transcendental functions in
  `opr_sim`/`opr_core` (ADR 0006). `Scene.FingerprintIsStableAcrossMachines` pins a frame.
- **No ROS in `opr_sim`, `opr_core`, `opr_v4l2`, `opr_shm`.** Nodes are thin wrappers.
- **A shared-memory slot is read only under lease and trusted only after `validate()`.**
- **The camera streams only while active**, and the frame sequence in `FrameInfo` never restarts.
- **Nothing blocks the executor on hardware**: the camera's capture thread owns the device;
  diagnostics read a snapshot.
- Parameters come from `config/pipeline.yaml`, which is generated from `visionbox.ini` by
  `tools/ini_to_params.py`. `opr_core` defaults equal the field values (`tests/test_tools.py`).

## Where things are

| Package | What |
|---|---|
| `opr_sim` | deterministic camera simulator with ground truth (no ROS) |
| `legacy_visionbox` | the frozen monolith the migration started from |
| `opr_core` | rectify, detect, track, telemetry codec; `opr_replay`, `opr_evaluate` (no ROS) |
| `opr_v4l2` | V4L2 mmap capture driver, simulated device, `opr_v4l2_probe` (no ROS) |
| `opr_shm` | leased shared-memory image slots (no ROS) |
| `opr_msgs` | messages |
| `opr_shm_transport` | image_transport `shm` plugin and the dual raw/shm publisher |
| `opr_pipeline` | lifecycle camera, rectify, detector, tracker, telemetry bridge, health; launch |
| `opr_bench` | transport and pipeline benchmarks |
