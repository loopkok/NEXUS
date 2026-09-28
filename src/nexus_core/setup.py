from glob import glob
from setuptools import find_packages, setup

setup(
    name="nexus_core",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/nexus_core"]),
        ("share/nexus_core", ["package.xml"]),
        ("share/nexus_core/profiles", glob("profiles/*.json")),
        ("share/nexus_core/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={"console_scripts": [
        "nexus_profile = nexus_core.cli:main",
        "nexus_command_mux = nexus_core.command_mux_node:main",
        "nexus_driver_manager = nexus_core.driver_manager_node:main",
        "nexus_joint_bridge = nexus_core.joint_bridge_node:main",
        "nexus_input_bridge = nexus_core.input_bridge_node:main",
        "nexus_nero_driver = nexus_core.nero_driver_node:main",
        "nexus_nero_teleop = nexus_core.nero_teleop_node:main",
        "nexus_orbbec_camera = nexus_core.orbbec_camera_node:main",
        "nexus_fake_driver = nexus_core.fake_driver_node:main",
        "nexus_pipeline = nexus_core.pipeline:main",
        "nexus_import_nero = nexus_core.import_nero:main",
        "nexus_import_nero_lerobot = nexus_core.import_nero_lerobot:main",
        "nexus_policy = nexus_core.policy_node:main",
        "nexus_gpu_job = nexus_core.remote_jobs:main",
    ]},
)
