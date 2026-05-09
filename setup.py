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
            'coordinate_dictionary_node = kinova_interface.coordinate_dictionary_node:main',
            'json_parser_node = kinova_interface.json_parser_node:main',
            'environment_mapping_node = kinova_interface.environment_mapping_node:main',
            'test_script_interface = kinova_interface.test_script_interface:main',
        ],
    },
)
