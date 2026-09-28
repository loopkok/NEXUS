from setuptools import setup, find_packages

package_name = "nero_dual_data_collect"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/data_collect.launch.py"]),
        ("share/" + package_name + "/config", ["config/data_collect.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="xnero",
    maintainer_email="user@example.com",
    description="Dual-arm dual-hand teleoperation data collection system",
    license="MIT",
    entry_points={
        "console_scripts": [
            "data_collect_node = nero_dual_data_collect.data_collect_node:main",
            "align_data = nero_dual_data_collect.align_data:main",
            "convert_to_lerobot = nero_dual_data_collect.convert_to_lerobot:main",
            "replay_episode = nero_dual_data_collect.replay_episode:main",
        ],
    },
)
