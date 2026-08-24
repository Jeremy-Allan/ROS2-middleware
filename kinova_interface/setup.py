import os
from glob import glob
from setuptools import setup

package_name = 'kinova_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'data', 'configs', 'env'), glob('data/configs/env/*.json')),
        (os.path.join('share', package_name, 'recipes'), glob(os.path.join('recipes', '*.json'))),
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeremyallan',
    maintainer_email='jeremyallan@todo.todo',
    description='Kinova Kortex ROS2 Middleware Interface',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hardware_interface_client = kinova_interface.hardware_interface_client:main',
            'environment_mapping_node = kinova_interface.environment_mapping_node:main',
            'json_parser_node = kinova_interface.json_parser_node:main',
            'telemetry_node = kinova_interface.telemetry_node:main',
            'cv_detection_node = kinova_interface.cv_detection_node:main',
        ],
    },
)