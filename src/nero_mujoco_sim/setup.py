from glob import glob
import os

from setuptools import find_packages, setup

package_name = "nero_mujoco_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "numpy", "mujoco>=3.2"],
    zip_safe=True,
    maintainer="NEXUS",
    maintainer_email="maintainer@example.com",
    description="MuJoCo physics simulator for the NEXUS Nero and XHand assembly",
    license="MIT",
    entry_points={
        "console_scripts": [
            "nero_mujoco_sim_node = nero_mujoco_sim.mujoco_sim_node:main",
        ],
        "nexus.driver_adapters": [
            "nero_mujoco = nero_mujoco_sim.nexus_adapter:driver_adapter",
        ],
    },
)
