from glob import glob
import os

from setuptools import find_packages, setup

package_name = "astral_data_collect"


def _existing(paths):
    return [p for p in paths if os.path.isfile(p)]


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    include_package_data=True,
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), _existing(glob("config/*"))),
        (os.path.join("share", package_name, "launch"), _existing(glob("launch/*.launch.py"))),
    ],
    install_requires=["setuptools", "numpy", "h5py"],
    zip_safe=False,
    maintainer="loopkok",
    maintainer_email="loopkok@todo.todo",
    description="Astral VLA data collection, alignment, validation, LeRobot v2.1 export, Rerun replay",
    license="MIT",
    entry_points={
        "console_scripts": [
            "data_collect_node = astral_data_collect.data_collect_node:main",
            "keyboard_controller = astral_data_collect.keyboard_controller:main",
            "align_data = astral_data_collect.align_data:main",
            "validate_data = astral_data_collect.validate_data:main",
            "convert_to_lerobot = astral_data_collect.convert_to_lerobot:main",
            "replay_rerun = astral_data_collect.replay_rerun:main",
        ],
    },
)
