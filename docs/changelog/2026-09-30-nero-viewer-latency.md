# 2026-09-30 — Restore Nero viewer real-time teleoperation

- Reproduced a desktop-viewer slowdown missed by the earlier headless tests:
  rendering every physics step reduced real-time factor to about 0.44 and
  limited four command streams to about 36 Hz each.
- Separated ROS reception from physics, synchronized shared data, retained only
  the latest final command, capped viewer sync, and used absolute step deadlines.
  The same visible test now runs at 500 Hz/real-time factor 1.00 with approximately
  96–100 Hz command reception and 100 Hz feedback. The physical model and IK core
  are unchanged.
- Reject expired stamped commands without resetting the timeout watchdog; added
  regression tests for expiry, queued pre-home commands, and estop. Wait for the
  native viewer's owner thread at shutdown to avoid GL teardown races.
- Added safe runtime probes, selectable synthetic input frequency, and arm IK
  diagnostics. Verified the full 72 Hz synthetic Quest-to-viewer path and
  deliberate input-loss pause. Detailed measurements and boundaries are in
  the [test report](../test_logs/2026-09-30-nero-viewer-latency/README.md).
