from setuptools import find_packages, setup

package_name = 'tb3_fortress_sim'

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
    maintainer='Lakshya Agarwal',
    maintainer_email='100lakshyaagarwal@gmail.com',
    description='Ignition Fortress + TurtleBot3 bringup for ROS2 Humble',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
