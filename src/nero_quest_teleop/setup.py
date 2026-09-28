import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'nero_quest_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'nero_teleop_node = nero_quest_teleop.nero_teleop_node:main',
            'ik_solver_node = nero_quest_teleop.ik_solver_node:main',
            'keyboard_vr_sim = nero_quest_teleop.keyboard_vr_sim:main',
            'test_ik_solver = nero_quest_teleop.test_ik_solver:main',
            'test_dataflow = nero_quest_teleop.test_dataflow:main',
            'test_safe_teleop = nero_quest_teleop.test_safe_teleop:main',
            'test_vr_mapping = nero_quest_teleop.test_vr_mapping:main',
        ],
    },
)
