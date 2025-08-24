from setuptools import setup
import os
from glob import glob

package_name = 'catalyst_algorithms'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CATALYST Team',
    maintainer_email='your.email@example.com',
    description='CATALYST Algorithm Plugins - A* path planning converted from MATLAB',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'astar_plugin = catalyst_algorithms.astar_plugin:main',
            'cost_calculators = catalyst_algorithms.cost_calculators:main',
            'collision_detection = catalyst_algorithms.collision_detection:main',
        ],
    },
)
