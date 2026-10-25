# Bringing the pipeline up on a vehicle

What to check, in order, the first time on new hardware or after a camera or image change. Each
step names the command and what a good answer looks like.

1. **The camera delivers what the calibration expects.**
   `opr_v4l2_probe /dev/video0 10 640 480` (inside the image: `ros2 run` is not needed, it is in
   `/opt/opr/lib/opr_v4l2/`). Good: about 30 fps, `dropped 0`, timestamps "start of exposure".
   "dequeue time" means the driver gives no timestamps: latency figures will include readout.
   "device offers WxH" means the mode is not supported: fix the camera, not the parameter.

2. **The vehicle's own calibration is loaded.**
   `tools/ini_to_params.py /path/to/that/vehicle/visionbox.ini > pipeline.yaml` and deploy it.
   The converter refuses unknown or missing settings; do not edit the yaml by hand.

3. **Containers can share memory with the camera.**
   The detector container needs the camera's IPC namespace (`ipc: service:camera` in
   docker/compose.yaml). Check its log for `reading image_rect in place`. `falling back to raw`
   means it cannot see the segment: it still works, at raw-transport cost (reports/benchmarks).
   Keep `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` on containers that share the host network but not
   /dev/shm, or Fast DDS's own shared-memory transport loses messages between them silently.

4. **Bring-up order.** With a lifecycle manager, activate the camera after the detector, tracker
   and bridge are up (`camera_autostart:=false`). Frames published before a subscriber is
   discovered are lost whatever the QoS.

5. **Health is green and means it.** `ros2 topic echo /perception/health`: level 0 with a track
   age around one to two frame periods. Pull the camera cable: within half a second level 2,
   `worst: tracks` or the camera's diagnostics; plug it back: level 0 again without a restart.

6. **The ground station sees the same thing as before.** Run visionbox and the pipeline on the same
   recorded flight (`--replay` / `sim://`) and compare the recorded telemetry files; for a live
   check, record both on the vehicle (`visionbox --record`, `record_path:=`) during a ground run.

7. **Thermal and CPU headroom.** Run `opr_bench/scripts/run_pipeline.py` on the vehicle with the
   rest of the autonomy stack running. Compare with reports/benchmarks/pipeline.md (measured on a
   cloud VM, not a Jetson); on Jetson also watch `tegrastats` for throttling during a long run.
