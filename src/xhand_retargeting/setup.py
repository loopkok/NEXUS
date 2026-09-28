import os
from glob import glob
from setuptools import find_packages, setup

package_name = "xhand_retargeting"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yml") + glob("config/*.yaml")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.urdf")),
        (os.path.join("share", package_name, "urdf", "meshes"), glob("urdf/meshes/*")),
    ],
    install_requires=["setuptools", "numpy"],
    maintainer="loopkok",
    maintainer_email="loopkok@todo.todo",
    description="XHand retargeting from Quest3 VR hand mocap data",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "xhand_dex_retargeting_node = xhand_retargeting.xhand_dex_retargeting_node:main",
            "test_hand_publisher = xhand_retargeting.test_hand_publisher:main",
            "test_direct_control = xhand_retargeting.test_direct_control:main",
        ],
    },
)
