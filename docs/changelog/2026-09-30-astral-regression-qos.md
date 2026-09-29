# 2026-09-30 — Astral regression and QoS verification

- Finished the Astral dry-run regression through the Web NEXUS entry point,
  including driver lifecycle, arm reanchor, recording, episode quality checks,
  pi0.5 export, and ACT export. The episode passed with no quality warnings.
- Checked live Astral wrist topics and found BEST_EFFORT/VOLATILE compatibility
  across the Quest source, input bridge, and arm IK subscriber. Hand input also
  reached the bridge and gripper. Wuji retargeting ran at 50 Hz without QoS
  warnings; the prior hand-landmark QoS fix is recorded in commit `420df88`.
- Checked Nero dual-arm and dual-XHand MuJoCo with synthetic Quest input. Both
  arms and hands received commands, reached `TELEOP`, and showed measured joint
  motion. XHand retargeting ran at 100 Hz and the hand bridges forwarded at
  50 Hz, with zero invalid commands and no QoS warnings.
- No runtime code change was needed for this verification. Detailed evidence
  and test limits are in
  [`docs/test_logs/2026-09-30-astral-regression-qos.md`](../test_logs/2026-09-30-astral-regression-qos.md).
