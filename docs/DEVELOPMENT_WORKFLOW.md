# NEXUS development and deployment workflow

The GitHub repository `loopkok/NEXUS`, local `main`, and the deployment checkout
`/home/loopkok/NEXUS` use `origin/main` as the only source of code. Changes are
made locally, pushed to GitHub, then pulled and tested on the deployment host.
Do not develop uncommitted source changes in the deployment checkout.

## Local change and publish

From the NEXUS workspace root:

```bash
git switch main
git pull --ff-only origin main
# edit code and run the relevant tests
git diff --check
git status --short
git add <changed files>
git commit -m "<change summary>"
git push origin main
git rev-parse HEAD
```

Keep generated robot data, checkpoints, and credentials outside Git. Commit
profiles only when they contain no site secrets or unverified hardware settings.

## Deployment host update and simulation gate

On the deployment host:

```bash
cd /home/loopkok/NEXUS
git status --short --branch
git remote set-url origin git@github.com:loopkok/NEXUS.git
git fetch origin
git pull --ff-only origin main
git rev-parse HEAD
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Use the host's configured GitHub SSH key. HTTPS ref listing worked, but the
host's HTTPS pack fetch stalled even with HTTP/1.1; GitHub SSH authentication
and fast-forward pulls were verified. For a new host checkout, clone with
`git clone git@github.com:loopkok/NEXUS.git /home/loopkok/NEXUS`. Confirm that
the host commit hash exactly matches the local commit that was pushed. If the
checkout is dirty, preserve and review the changes before updating; do not reset
or overwrite them.

Run the simulation in an isolated ROS domain. Disable physical input and camera
drivers when synthetic Quest 3 and camera topics are used:

```bash
export ROS_DOMAIN_ID=73 ROS_LOCALHOST_ONLY=1
ros2 launch nexus_core system.launch.py \
  profile:=nero_dual_xhand_mujoco dry_run:=false \
  with_inputs:=false with_cameras:=false \
  with_recording:=true with_policy:=true backend_type:=stub \
  data_root:=/home/loopkok/NEXUS/data/simulation session:=smoke
```

In another sourced shell, run the synthetic recording and policy checks:

```bash
python3 scripts/nexus_sim_acceptance.py \
  --profile src/nexus_core/profiles/nero_dual_xhand_mujoco.json
python3 scripts/nexus_sim_acceptance.py \
  --profile src/nexus_core/profiles/nero_dual_xhand_mujoco.json \
  --phase policy
```

Use a new `data_root` and `session` for each recording. Then align and validate
the episode and export the training layout. Save command output and the profile
hash under `docs/test_logs/` before publishing the test report. The full test
suite currently contains legacy lint failures and CAN-dependent `pyAgxArm`
demos; avoid running those demos on a host with live robot CAN interfaces until
they are marked as hardware tests.

## Real hardware gate

Real hardware is a separate deployment step after the matching profile passes
the applicable simulation, data, and model checks in
[`NEXUS_ACCEPTANCE.md`](NEXUS_ACCEPTANCE.md). Verify that the selected profile
names the intended physical drivers and device IDs, then use the site-approved
enable and emergency-stop procedure. Never reuse the MuJoCo profile for real
hardware or start physical drivers as part of a synthetic-input test.
