# Dependency boundary

NEXUS source is based on the exact Astral and XNero commits in `MIGRATION_MANIFEST.json`. The migrated `pyAgxArm` source and XHand shared library are vendored. Quest uses only Astral's `quest3_hand_mocap` ROS package, avoiding a duplicate package name in colcon.

The repository does not vendor ROS 2 Humble, the Astral/Wuji vendor SDKs, Orbbec `pyorbbecsdk`, LeRobot or OpenPI. Their versions and device firmware must be captured from the actual robot and GPU hosts before real acceptance. This checkout was prepared on Windows without those hosts, so a truthful binary lockfile cannot be generated here. Record `ros2 doctor`, `colcon list`, `python3 -m pip freeze`, vendor library hashes, firmware and CAN/serial/camera IDs in the site acceptance log. Freeze those outputs together with the profile SHA256 for deployment.

The GPU host needs Python packages used by `astral_data_collect` (HDF5, NumPy, image/video encoding and LeRobot) and, for pi0.5, a profile-specific OpenPI configuration and base checkpoint. `NEXUS_GPU_PYTHON` must point to the installed NEXUS Python environment. The ACT script path must be provided as `NEXUS_GPU_ACT_SCRIPT`; the script resolves `LEROBOT_TRAIN` on the GPU host.
