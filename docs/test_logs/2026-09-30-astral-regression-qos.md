# Astral regression and QoS verification — 2026-09-30

Tested source revision: `8c2a0700db2d9fedefaa7a6c334165084c82d334`. The local
checkout and deployment checkout were at this same revision before the run.
ROS 2 Humble tests ran on the deployment host with localhost-only ROS domains,
synthetic Quest input, and simulation/dry-run drivers. No physical robot,
Quest headset, or camera was used.

## Astral functionality and data path

- Started `astral_gripper_wuji` through the Web NEXUS API with dry-run drivers.
  Driver ready, enable, and home services succeeded. The Web teleop start request
  reached both arm IK subscribers and reanchored them.
- Recorded a 4-second episode. The recorder saved 3,007 stream samples and 363
  images across the base, left-wrist, and right-wrist cameras. Its final episode
  report passed: 1 pass, 0 warnings, 0 failures, 0 quarantined episodes.
- The pipeline completed with exit code 0. pi0.5 export contains 131 frames at
  30 Hz, state dimension 35, and the three expected camera keys. ACT export also
  completed. The raw `robot_data.h5` and `camera_data.h5` remained in the source
  session; the generated outputs were under `/tmp/nexus_astral_regression_data`.
- Arm IK and Wuji retargeting were active during the capture. The arm log showed
  about 4–6 ms typical IK loop time, and the Wuji retargeter reported 50 Hz.

## QoS checks

`ros2 topic info --verbose` and launch logs were checked while the Astral stack
was running with synthetic input:

| Stream | Publisher QoS | Subscriber QoS | Result |
| --- | --- | --- | --- |
| Quest left wrist → NEXUS input bridge | BEST_EFFORT / VOLATILE | BEST_EFFORT / VOLATILE | Compatible |
| NEXUS wrist bridge → left arm teleop IK | BEST_EFFORT / VOLATILE | BEST_EFFORT / VOLATILE | Compatible |
| Quest left hand → NEXUS input bridge | BEST_EFFORT / VOLATILE | BEST_EFFORT / VOLATILE | Compatible |
| NEXUS hand bridge → left gripper pinch input | BEST_EFFORT / VOLATILE | BEST_EFFORT / VOLATILE | Compatible |

No incompatible-QoS warnings or ROS error lines appeared in the Astral launch
log. The same run also exercised the right Wuji retargeter, which reported a
50 Hz stream. The previous Wuji hand-landmark reliability mismatch is fixed in
`420df88` by using sensor-data QoS on its input subscription.

## Nero dual-arm, dual-XHand QoS and motion

Started `nero_dual_xhand_mujoco` through the Web NEXUS API with all four
components using the MuJoCo driver. Ready, enable, home, and teleop reanchor
succeeded. During a 5-second synthetic Quest motion window:

- The arm feedback dimensions were 7 per arm; hand feedback dimensions were 12
  per hand. All four components became ready and control entered `TELEOP`.
- Maximum measured joint changes were 0.07789 rad (left arm), 0.07670 rad
  (right arm), 1.89101 rad (left hand), and 1.89103 rad (right hand).
- XHand retargeting reported 100 Hz. The two NEXUS hand bridges forwarded
  approximately 40 Hz during startup, then 50 Hz; invalid commands were 0,
  maximum callback gaps were 23.6–26.4 ms, and maximum publish times were
  0.1–0.3 ms.
- MuJoCo physics ran at 477.6 Hz with a 0.96 real-time factor. No incompatible
  QoS warnings or ROS error lines appeared in the launch log.

**Conclusion:** the QoS mismatch seen earlier on the Astral Wuji path is fixed.
This validation found no corresponding QoS mismatch in the Nero dual-arm,
dual-XHand path. Synthetic input reached both arm IK nodes and both XHand
retarget/bridge paths, and all four simulated components moved.

## Scope limits

Astral used dry-run drivers because the deployment environment did not provide
the Astral vendor SDK. The Nero run used MuJoCo. These runs validate ROS graph
compatibility, teleop routing, simulated motion, recording, quality checks, and
dataset export; they do not certify physical driver behavior or real-device
latency. Training jobs and real Quest/camera capture were not part of this run.
