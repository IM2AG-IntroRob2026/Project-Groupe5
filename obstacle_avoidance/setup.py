from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'obstacle_avoidance'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Includes the launch folder and the specific explorer launch file
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='draken',
    maintainer_email='slrosales@uninorte.edu.co',
    description='Autonomous obstacle avoidance and docking behavior for the iRobot Create 3.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # executable_name = package_name.filename:main_function
            'explorer = obstacle_avoidance.create3_controller:main',
            'keyboard_handler = obstacle_avoidance.keyboard_handler:main',
        ],
    },
)