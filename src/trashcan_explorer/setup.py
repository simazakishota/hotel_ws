from setuptools import find_packages, setup

package_name = 'trashcan_explorer'

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
        'console_scripts': [
                        'trashcan_explorer_node = trashcan_explorer.trashcan_explorer_node:main',
                        'trashcan_filter_node = trashcan_explorer.trashcan_filter_node:main',
        ],
    },
)
