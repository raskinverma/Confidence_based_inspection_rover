from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rover_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='raskin',
    maintainer_email='vermaraskin@gmail.com',
    description='Defect detection, pose projection, and logging',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'defect_detector = rover_perception.defect_detector:main',
            'pose_projector = rover_perception.pose_projector:main',
            'defect_logger = rover_perception.defect_logger:main',
        ],
    },
)
