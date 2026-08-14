from glob import glob
import os

from setuptools import find_packages, setup

package_name = "astral_quest_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    package_data={
        "astral_quest_teleop": [
            "robot/urdf/*.urdf",
            "robot/RobotMain_URDF/meshes/*",
        ],
    },
    include_package_data=True,
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "numpy", "scipy"],
    zip_safe=False,
    maintainer="loopkok",
    maintainer_email="loopkok@todo.todo",
    description="Astral Quest3 dual-arm teleop (DH + URDF IK)",
    license="MIT",
    entry_points={
        "console_scripts": [
            "astral_teleop_node = astral_quest_teleop.astral_teleop_node:main",
            "astral_teleop_arm_node = astral_quest_teleop.astral_teleop_arm_node:main",
            "ik_solver_node = astral_quest_teleop.ik_solver_node:main",
            "keyboard_vr_sim = astral_quest_teleop.keyboard_vr_sim:main",
            "test_ik_solver = astral_quest_teleop.test_ik_solver:main",
            "test_dh_urdf_fk = astral_quest_teleop.test_dh_urdf_fk:main",
            "fit_dh_from_urdf = astral_quest_teleop.fit_dh_from_urdf:main",
            "test_vr_mapping = astral_quest_teleop.test_vr_mapping:main",
            "test_dataflow = astral_quest_teleop.test_dataflow:main",
            "test_safe_teleop = astral_quest_teleop.test_safe_teleop:main",
        ],
    },
)
