from setuptools import setup
import os
from glob import glob

package_name = 'catalyst_core'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CATALYST Team',
    maintainer_email='your.email@example.com',
    description='CATALYST Core Platform - XX (work)-inspired layered architecture',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'platform_manager = catalyst_core.platform_manager:main',
            'plugin_registry = catalyst_core.plugin_registry:main',
            'configuration_manager = catalyst_core.configuration_manager:main',
        ],
    },
)
