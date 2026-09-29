# Host reconciliation and verification — 2026-09-29

The tested source revision was `adf722232f2715372fccdf033fe6dfc3e61cbd72`
(`build: declare MuJoCo simulator test dependencies`). The commit added only
the simulator test dependency declaration on top of the Quest pose-mapping fix.
This report and its raw logs are documentation-only follow-up changes.

## Source reconciliation

- Local `main`, GitHub `origin/main`, and the clean host candidate checkout were
  all at `adf722232f2715372fccdf033fe6dfc3e61cbd72` before this report was
  prepared.
- Compared the old host working tree to the NEXUS migration commit after
  normalizing CRLF/LF line endings. 791 checked source files matched. The five
  remaining paths were old/superseded host versions of the Quest mapping and
  policy regression test, plus a newline-only package file difference. The
  current candidate contains their later committed replacements. The old host
  checkout and its data are retained intact during the final path switch.
- HTTPS ref listing works, but an HTTPS pack fetch stalls on this host even
  with HTTP/1.1. GitHub SSH authentication succeeded, and the candidate
  checkout fast-forwarded to the published documentation/test-log commit over
  SSH. The canonical host checkout will use the SSH remote.

## Passed on the Ubuntu 22.04 / ROS 2 Humble host

- Full workspace build: **30 packages finished**.
- Relevant pure/unit tests: **28 passed** across NEXUS contracts, Nero MuJoCo
  model construction, and Quest pose mapping (`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`).
- Nero dual-arm + dual-XHand simulation, with synthetic Quest 3 and JPEG camera
  input, passed recording acceptance. Final command topics each had one
  publisher. Arm command flow was 94.5–94.6 Hz; all three synthetic camera
  streams produced 757 frames. Link7 translation tracking was 9.83–9.86 mm
  against 10.4 mm expected, input/output correlation 0.962–0.964, and measured
  rotation drift 0.035°. Input-to-candidate latency was 3.5/10.0 ms median and
  6.2/12.8 ms p95 for left/right. Wrist loss paused arbitration in 856 ms.
- The recording produced one 15.07 s episode with 452 aligned frames at 30 Hz,
  state dimension 38, and base/left-wrist/right-wrist camera roles. The raw
  quality check reported **1 pass, 0 warnings, 0 failures**.
- ACT LeRobot v3 export completed with 452 frames; its structural checks passed
  for statistics, camera dimensions, task metadata, Parquet, and video decoding.
- The stub policy/HITL run passed POLICY → HUMAN takeover → POLICY return → IDLE.
- No physical robot driver or real camera was launched. The simulator used
  `ROS_DOMAIN_ID=73`, `ROS_LOCALHOST_ONLY=1`, and synthetic sensor publishers.

## Remaining test gates

- A full `colcon test` is **not green**. The legacy Quest package still fails
  flake8/pep257 checks, and the imported XHand ROS driver has copyright,
  flake8, CMake line-length, and uncrustify failures. `pyAgxArm`'s demo tests
  also fail because `can0` is down and `can_nero_left` does not exist on this
  test host. They were not retried after that full test run, to avoid repeated
  CAN access attempts.
- The host does not have `lerobot-train` or an OpenPI training environment, so
  this sync verified ACT export and stub policy control, not a new ACT/pi0.5
  training job or a real checkpoint service handshake.
- This is simulated acceptance only. Real driver enablement, real camera
  capture, and real-robot release checks remain separate gates.

## Raw logs

- [`nexus_adf7222_full_build.log`](nexus_adf7222_full_build.log)
- [`nexus_adf7222_full_test.log`](nexus_adf7222_full_test.log)
- [`nexus_adf7222_sim_record.json`](nexus_adf7222_sim_record.json)
- [`nexus_adf7222_policy.json`](nexus_adf7222_policy.json)
- [`nexus_adf7222_align.log`](nexus_adf7222_align.log)
- [`nexus_adf7222_validate.log`](nexus_adf7222_validate.log)
- [`nexus_adf7222_act_export.log`](nexus_adf7222_act_export.log)
