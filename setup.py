from setuptools import setup, find_packages

package_name = 'drone_direction'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),  # Automatically find sub-packages
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/direction_drone_launch.py']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Elad Par',
    maintainer_email='eladpar@gmail.com',
    description='ROS2 package for controlling drone direction using AI models.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'accurate_model = accurate_model.accurate_model:main',
            'heuristic_model = heuristic_model.heuristic_model:main',
            'controller_node = controller.controller_node:main',
        ],
    },
)
