# Nero viewer latency investigation — 2026-09-30

The reported Quest input reached about 72 Hz per hand with 0.5–0.6 ms
receive-to-publish time, while visible MuJoCo ran at a 0.38–0.41 real-time
factor. The previous headless acceptance did not exercise this graphics
bottleneck. The issue was reproduced on the deployment desktop using the
same dual-arm/dual-XHand model.

## Cause and changes

The old simulator synchronized the viewer on every 2 ms physics step and
executed at most one ROS callback per step. Four 100 Hz command subscriptions
plus the state timer outpaced callback processing once rendering slowed the
loop. Incoming target streams accumulated in the sensor-data depth-5 queues.

Runtime change `f125ebb3405c0f76ea086ab6c1250b6408f64e79` moves ROS callbacks
to a dedicated single-thread executor, locks shared simulator/lifecycle state,
uses depth-1 best-effort final-command subscriptions, and limits viewer sync
to 30 Hz. Absolute physics deadlines recover short rendering stalls with
bounded catch-up. Expired stamped commands are rejected without refreshing
the command watchdog. Viewer threads finish before GL teardown.

The URDF model, 2 ms physics step, servo gains, joint limits, profile hash,
Nero IK algorithm, smoothing, and control safety transitions were preserved.
The Nero wrapper now logs solve rate, solve p95, callback input age, failed
solutions, and workspace clips, so future apparent freezes can be attributed
to an infeasible target or control pause.

## Same desktop, same target-stream benchmark

The safe runtime probe requests 100 Hz targets on all four components. Both
runs use the same model and an open passive viewer, and exclude window startup,
shutdown, and initial DDS discovery from stream metrics. The old run executes
the simulator module from source `2995b59` without editing the deployment tree;
the fixed run uses `c2fd542` (runtime implementation from `f125ebb`).

| Measurement | Previous code | Fixed code |
| --- | ---: | ---: |
| Physics rate | 218.5 Hz | 499.6 Hz |
| Real-time factor | 0.437 | 0.999 |
| Command callback rate per component | 36.2 Hz | 95.6–100.0 Hz |
| State feedback rate per component | 36.2–36.3 Hz | 100.0 Hz |
| Command receipt age p95 | 48.8–49.4 ms | 3.6–8.2 ms |
| Left/right arm servo phase lag | 351 / 345 ms | 111 / 113 ms |
| Left/right hand servo phase lag | 172 / 105 ms | 49 / 42 ms |
| Viewer sync rate | 217.4 Hz | 25.1 Hz |

Servo phase lag is estimated from a 0.8 Hz, 0.02 rad joint sine. It measures
the driver and actuator response, not complete Quest-to-screen latency. The
remaining arm lag is consistent with the unchanged physical actuator model;
the original Astral viewer directly updates joint positions. The probe uses
small latest-value commands, so occasional best-effort drops do not create an
old-command queue.

The isolated probe pre-enables simulated components before opening the native
window. Startup command-timeout lines can appear while window creation delays
the first physics/executor pass. This initial interval is excluded from the
steady metrics. The full Web test follows ready/enable/home ordering and shows
no input timeout until the deliberate source shutdown.

## Complete Web teleop graph with visible viewer

Using the same unmodified simulation profile, synthetic Quest wrists and
landmarks were published at 72 Hz per side in an isolated ROS domain. The Web
API started MuJoCo, checked readiness, enabled, homed, and reanchored both arms.

- Physics: 499.5–499.9 Hz, real-time factor 1.00, actual viewer sync about 25 Hz.
- Arm IK: 50.0 Hz per arm; solve p95 8.5–8.9 ms; failed solves 0; workspace
  clips 0 in the observed steady motion window.
- XHand retargeting: about 143 Hz combined; bridges: 72.0 Hz per hand, invalid
  commands 0, maximum callback gaps 16.4–17.2 ms.
- All 20 half-second snapshots remained in TELEOP. Feedback was fresh with
  dimensions 7/7/12/12. After deliberate input shutdown, PAUSED was observed
  1.5 seconds later, with a stale-command fault. Arm timeout logs at that point
  are expected dropout evidence.
- Build succeeded. The model, runtime command safety, and core suite passed
  27 tests at source `adc6ae26f28cfe818cabcdbddb3f4b477330a867`:
  `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/nero_mujoco_sim/test src/nexus_core/test -q`.

The raw logs below contain the benchmark, graph, source-rate, build, and test
results. The runs used synthetic Quest data and physical MuJoCo dynamics;
no physical robot driver, camera, or real headset was operated. Arbitrary real
hand motions and unreachable poses still require a real Quest retest. The
new IK diagnostics identify rejected targets explicitly.

## Archived logs

- [Previous visible runtime](nero_viewer_baseline_steady.log)
- [Fixed visible runtime](nero_viewer_fixed_steady.log)
- [Complete Web teleop graph and dropout](nero_full_graph_viewer_check.log)
- [72 Hz synthetic Quest source](nero_72hz_source.log)
- [Targeted build](nero_runtime_build.log)
- [Model, command safety, and core tests](nero_runtime_tests.log)
