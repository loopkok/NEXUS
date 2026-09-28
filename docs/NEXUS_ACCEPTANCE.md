# Site acceptance checklist

## Baseline and static gates

- [ ] Record original Astral and Nero topic graph, `ros2 topic hz`, ordered joint names, camera role/serial mapping, physical stop behavior and latency. Save original repository commits from the migration manifest.
- [ ] Record ROS/SDK/LeRobot/OpenPI versions and vendor binary hashes from the actual robot and GPU hosts; make a deployment lock from those observations.
- [ ] Run `nexus_profile` for every schema v2 site profile; reject unregistered adapters, incompatible component kinds, changed native joint order/dimension, missing input frames/topics, duplicate camera roles and device identifiers. Confirm there is no robot-family selector and the printed SHA256 is the same in launch, episode metadata and model manifest.
- [ ] Check each final `components/*/joint_commands` has exactly one publisher (`nexus_command_mux`). Kill duplicate publishers before enabling hardware.
- [ ] Exercise `/nexus/<instance>/drivers/ready|enable|home|estop`; verify `ready` does not move hardware, unsupported home operations fail explicitly, and partial enable failure estops every selected adapter.
- [ ] Confirm selected source topics, camera device paths/serials and semantic roles come from the profile, then capture `ros2 node info`/`ros2 topic info -v` output as interface evidence.

## Dry run and data

- [ ] Start each profile with `dry_run:=true`, fake drivers and camera replay. Exercise start, pause, replay, HITL reanchor, release, timeout, disconnect, emergency stop and recovery.
- [ ] Collect Astral and Nero samples. Check raw stream timestamps, compressed JPEG decode, camera role correctness and real measured states (Astral gripper is marked as command echo).
- [ ] Run remote `sync_process`; inspect alignment and `validation_report.json`. Confirm bad episodes enter `quarantine`, and ACT/pi0.5 exports have the same `nexus_layout.json`.
- [ ] Import representative old Nero raw HDF5 and LeRobot episodes. Verify state/action segments and camera mapping manually and compare source hashes before and after.
- [ ] Run one short ACT and one short pi0.5 training job on the GPU host. Verify checkpoint layout stamps and remote model handshake; attempt to load a mismatched checkpoint and confirm rejection.

## Real hardware release gate

- [ ] Check mechanical/electrical fit and calibration of the exact Astral gripper/Wuji and Nero/XHand assemblies. Do not infer support for unvalidated cross-robot hand swaps.
- [ ] Confirm no command is sent before explicit enable; confirm local driver timeout, stale input, inference timeout and failed mode transition produce hold or physical disable as specified.
- [ ] With people clear of the workspace, demonstrate Astral and Nero individually: teleop → record → GPU sync/process → ACT/pi0.5 train → inference → human takeover → return to policy. Compare frequency, latency and fault logs with baseline.
- [ ] Confirm physical emergency stop on arm drivers and Wuji's native disable service. XHand currently has only a software command gate; verify its hardware stop procedure before unattended operation.

The built-in profiles are **not** marked as site accepted by this repository. Keep a real profile disabled until every applicable safety and data-layout gate above has recorded evidence.
