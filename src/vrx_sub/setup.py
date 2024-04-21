from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vrx_sub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),

        (os.path.join('share', package_name), glob('vrx_sub/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='guildstudent',
    maintainer_email='aritra.oceana@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_planner = vrx_sub.mission_planner:main',
            'path_planner = vrx_sub.path_planner:main',
            'control = vrx_sub.control:main',
            'robot_loc = vrx_sub.robot_loc:main',
        ],
    }
)
