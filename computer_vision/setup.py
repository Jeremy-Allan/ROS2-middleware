import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'computer_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=[],
    zip_safe=True,
    maintainer='Enoch',
    maintainer_email='enoch@example.com',
    description='YOLO + depth perception 3D bounding boxes for depth sensing cam',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = computer_vision.vision_node:main',
        ],
    },
)
