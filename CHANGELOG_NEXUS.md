# NEXUS Changelog

## 2026-09-29 — Reconcile local, GitHub, and deployment host

- Selected GitHub `origin/main` as the source of truth. The canonical host
  checkout `/home/loopkok/NEXUS` now tracks `main` over SSH; local and GitHub
  were verified at the same commit. The previous dirty host checkout and its
  data are preserved at `/home/loopkok/NEXUS_archive_b276065_20260929`.
- At source revision `c9953d36d9bf324edda901e35f8230a6ff766b4c`, rebuilt all 30
  ROS packages and reran 28 targeted tests from the final host path. MuJoCo
  recording, link7 tracking, dropout pause, data alignment/quality, ACT v3
  export, and stub-policy HITL results are in the [host reconciliation report](docs/test_logs/2026-09-29-host-reconciliation/README.md).
- Full `colcon test` still reports legacy Quest/XHand style and metadata lint
  failures and hardware-dependent `pyAgxArm` demo tests that require CAN
  interfaces. No robot driver, servo enable, or physical camera was launched.
  ACT/pi0.5 training was not repeated because this host has no LeRobot training
  command or OpenPI runtime.
- Added [the deployment workflow](docs/DEVELOPMENT_WORKFLOW.md): local edits
  push to GitHub, then the host fast-forwards over its verified GitHub SSH key
  and runs build/simulation gates. HTTPS pack fetching stalled on the host.

## 2026-09-29 — Correct Nero Quest frame contract

- Nero Quest wrist/controller input is the Nero `link7` flange pose. Teleoperation
  now anchors to measured `link7` FK and passes its target directly to the existing
  Nero IK solver; the IK core is unchanged and no flange-to-palm offset is applied
  to Quest targets.
- Moved the legacy XNero left/right rotation matrices to the Quest input adapter,
  selected by the Nero assembly profile. For `per_side` mode, that adapter applies
  the configured side rotation to the wrist pose once and emits stable side-specific
  frame IDs. `vr_to_arm_rot` is identity in NEXUS for both Nero arms. Generic
  `convert_to_robot` remains available for hand/body streams and is bypassed for
  these wrist/controller poses.
- Moved the URDF flange-to-XHand-palm transform out of teleop settings into
  `frame_transforms.link7_to_xhand_palm`; MuJoCo FK parity still checks this physical
  geometry independently from Quest teleoperation.
- Invalidated the previous Quest-to-TCP tracking and motion-amplitude metrics; they
  measured the wrong input/output frames. Raw prior logs remain unchanged and are
  marked superseded in the acceptance log. The corrected-hash MuJoCo rerun validates
  link7 tracking and the later host reconciliation run validates episode quality
  and ACT export. Real checkpoint validation remains pending.

## 2026-09-29 — Nero MuJoCo frame and lifecycle validation

- [Superseded frame semantics] The original run incorrectly used `tcp_offset` to
  remap Quest teleoperation targets; see the correction entry above.
- Moved the URDF flange-to-XHand-palm transform to `frame_transforms` for physical
  FK parity checks, and retained a MuJoCo-only J2 zero offset to reconcile
  motor-angle and URDF coordinates.
- Made measured homing tolerance a driver-adapter setting. MuJoCo permits 0.1 rad for its
  stable gravity-compensated home state; other adapters keep the 0.05 rad default.
- Prevented queued pre-homing hold commands from cancelling the MuJoCo home trajectory
  by comparing timestamps against the home request. The simulator homing watchdog is 45 s,
  beyond the manager's 30 s measured-convergence wait.
- Compared analytic FK, MuJoCo link7 and physical XHand TCP transforms on both arms across
  home, neutral and asymmetric probe poses. TCP position difference stayed below 30 μm and
  orientation difference below 0.001°.

## 2026-09-29 — Simulation acceptance and ACT runtime compatibility

- Built the NEXUS ROS 2 Humble workspace on `192.168.0.231`: all 30 packages passed;
  rebuilt `astral_policy_inference` after its runtime compatibility update.
- Added LeRobot 0.4.x loading through `lerobot.policies.factory` and the checkpoint's
  `PolicyProcessorPipeline` JSON files. The newer LeRobot processor API remains supported.
  Normalized action conversion now accepts Tensor and numpy outputs.
- Verified the ACT checkpoint trained for two CPU steps through the actual WebSocket server,
  NEXUS `RemoteBackend`, the ROS policy node, MuJoCo candidate topics and HITL handoff.
  The server returned finite 100×38 chunks with the matching frozen profile hash.
- Added regression coverage for the legacy LeRobot processor loader path.
- Full simulation, data, model and performance outcomes are recorded in
  [`docs/test_logs/2026-09-29-remote-simulation-acceptance.log`](docs/test_logs/2026-09-29-remote-simulation-acceptance.log).

## Earlier framework work retained

- Added `nero_mujoco_sim` and the `nero_dual_xhand_mujoco` profile from the XNero
  description package. The simulator exposes measured joint state through the NEXUS driver
  contract and builds the dual-arm, dual-XHand MJCF from URDF meshes.
- Added offline checks for profile loading, MJCF generation, actuator coverage, physics
  stepping, joint response, initial contact state and cube/table contact.
- Added a hardware-free ROS fixture for stable-framed Quest3 wrist/hand/body/Joy streams and
  compressed synthetic camera frames, with metrics for topic flow, arbitration, IK/FK TCP
  tracking, rotation drift, input latency, wrist-loss pause, recording, inference and HITL.
- Corrected XHand retargeter launch compatibility, ROS node field-name collisions, and
  sensor-data QoS for hand landmark input. Reduced redundant executor spins in MuJoCo.
- High-rate candidate commands use best-effort, keep-last depth-1 QoS. Candidate-only XHand
  retargeting uses a single-threaded executor and sensor-data QoS to sustain the simulated
  candidate stream without queue buildup.
