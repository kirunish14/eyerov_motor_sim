import os
from glob import glob

from setuptools import setup

package_name = 'eyerov_motor_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kirunish',
    maintainer_email='kirunish16@gmail.com',
    description='Motor control firmware simulator with ROS 2 telemetry bridge',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_sim_node = eyerov_motor_sim.motor_sim_node:main',
        ],
    },
)
