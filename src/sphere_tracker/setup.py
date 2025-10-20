from setuptools import setup, find_packages

package_name = 'sphere_tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shota',
    maintainer_email='shota@example.com',
    description='Red sphere 3D position tracker',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'red_ball_follower = sphere_tracker.red_ball_follower:main',
             'sphere_tracker = sphere_tracker.sphere_tracker:main',
             'red_ball_follower_action = sphere_tracker.red_ball_follower_action:main', 
             'test = sphere_tracker.test:main',
        ],
    },
)
