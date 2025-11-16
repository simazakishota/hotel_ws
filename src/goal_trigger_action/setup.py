from setuptools import find_packages, setup

package_name = 'goal_trigger_action'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='araishogo',
    maintainer_email='7521059@ed.tus.ac.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['goal_arrived_pointcloud_node = goal_trigger_action.goal_arrived_pointcloud_node:main',
        ],
    },
)
