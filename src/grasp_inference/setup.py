from setuptools import setup
import os
from glob import glob

package_name = 'grasp_inference'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='araishogo',
    maintainer_email='example@example.com',
    description='GraspNet inference ROS2 node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grasp_inference_node = grasp_inference.grasp_inference_node:main',
        ],
    },
)
