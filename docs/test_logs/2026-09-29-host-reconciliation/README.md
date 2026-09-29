# Deployment sync and verification — 2026-09-29

The final source revision built and exercised from the canonical host path was
`c9953d36d9bf324edda901e35f8230a6ff766b4c`. Local `main`, GitHub `origin/main`,
and `/home/loopkok/NEXUS` were verified at this same revision. Commits after
`adf7222` changed documentation, logs, and data ignore rules only; the full
workspace test result below is from the same functional source tree.

## Source reconciliation and host layout

- The original dirty checkout is preserved at
  `/home/loopkok/NEXUS_archive_b276065_20260929`. The canonical
  `/home/loopkok/NEXUS` is a clean `main` checkout with the SSH remote
  `git@github.com:loopkok/NEXUS.git`.
- Original `data` and `test_artifacts` remain accessible through symlinks from
  the canonical checkout into the archive. This run's raw episode and ACT
  export are saved under `/home/loopkok/NEXUS/data/simulation/`.
- Compared the old host working tree to the NEXUS migration commit after
  normalizing CRLF/LF line endings. 791 checked source files matched. The five
  remaining files were superseded Quest mapping/policy tests or a newline-only
  package-file difference; the final checkout contains their newer versions.
- HTTPS ref listing works, but HTTPS pack fetching stalls on this host. GitHub
  SSH authentication and `git pull --ff-only origin main` were verified.

## Final-path checks at `c9953d3`

- Clean-login full workspace build: **30 packages finished**.
- Targeted NEXUS contract, Nero MuJoCo model, and Quest pose-mapping tests:
  **28 passed** (`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`).
- Nero dual-arm + dual-XHand MuJoCo teleoperation with synthetic Quest 3 and
  JPEG camera input: **PASS**. Every final arm/hand command topic had one
  publisher. Arm command flow was about 95.5 Hz; synthetic camera counts were
  743/743/742. Link7 translation tracked 9.83/9.86 mm against 10.4 mm expected,
  with input/output correlation 0.967/0.962 and orientation drift 0.0355/0.0353°.
  Input-to-candidate latency p50/p95 was 13.4/19.4 ms left and 1.9/5.6 ms right.
  Wrist loss paused arbitration in 832 ms.
- The saved episode is 14.53 s, 436 frames at 30 Hz, state dimension 38, with
  base/left-wrist/right-wrist camera roles. Alignment reported no missing state
  blocks, state gaps, or distant image frames. Data quality reported **1 pass,
  0 warnings, 0 failures**.
- ACT LeRobot v3 export completed for 436 frames. Structural checks passed for
  statistics, camera dimensions, task metadata, Parquet, and video decoding.
- Stub-policy HITL passed POLICY → HUMAN takeover → POLICY return → IDLE.
- No real robot driver, servo enable, or physical camera launch occurred. These
  runs used `ROS_DOMAIN_ID=73`, `ROS_LOCALHOST_ONLY=1`, the MuJoCo profile, and
  synthetic sensor publishers.

## Remaining test gates

- The full `colcon test` run is **not green**. The legacy Quest package fails
  flake8/pep257 checks. The imported XHand ROS driver fails copyright, flake8,
  CMake line-length, and uncrustify checks. `pyAgxArm` demo tests failed before
  connecting because `can0` is down and `can_nero_left` does not exist. Those
  CAN tests were not repeated. No motor command was issued.
- The deployment host has no `lerobot-train` or OpenPI training environment.
  This verification covers ACT export and stub policy control, not a new
  ACT/pi0.5 training run or a real checkpoint service handshake.
- Physical camera capture and real-robot safety/release checks remain separate
  acceptance gates.

## Raw logs

Final-path logs from `c9953d3`:

- [`nexus_c9953d3_final_cleanenv_build.log`](final-c9953d3/nexus_c9953d3_final_cleanenv_build.log)
- [`nexus_c9953d3_final_targeted_tests.log`](final-c9953d3/nexus_c9953d3_final_targeted_tests.log)
- [`nexus_c9953d3_final_record.json`](final-c9953d3/nexus_c9953d3_final_record.json)
- [`nexus_c9953d3_final_policy.json`](final-c9953d3/nexus_c9953d3_final_policy.json)
- [`nexus_c9953d3_final_align.log`](final-c9953d3/nexus_c9953d3_final_align.log)
- [`nexus_c9953d3_final_validate.log`](final-c9953d3/nexus_c9953d3_final_validate.log)
- [`nexus_c9953d3_final_act_export.log`](final-c9953d3/nexus_c9953d3_final_act_export.log)

Earlier broad suite and preliminary candidate logs:

- [`nexus_adf7222_full_build.log`](nexus_adf7222_full_build.log)
- [`nexus_adf7222_full_test.log`](nexus_adf7222_full_test.log)
- [`nexus_adf7222_sim_record.json`](nexus_adf7222_sim_record.json)
- [`nexus_adf7222_policy.json`](nexus_adf7222_policy.json)
- [`nexus_adf7222_align.log`](nexus_adf7222_align.log)
- [`nexus_adf7222_validate.log`](nexus_adf7222_validate.log)
- [`nexus_adf7222_act_export.log`](nexus_adf7222_act_export.log)
