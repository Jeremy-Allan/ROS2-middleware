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
            'mover = kinova_interface.move_robot:main',
        ],
    },
)