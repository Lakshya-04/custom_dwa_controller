# from setuptools import find_packages, setup
# from glob import glob

# package_name = 'tb3_fortress_sim'

# data_files = [
#     ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
#     ('share/' + package_name + '/launch', glob('launch/*.py')),
#     ('share/' + package_name + '/models/turtlebot3_burger', glob('models/turtlebot3_burger/*')),
#     ('share/' + package_name + '/scripts', glob('scripts/*')),
# ]

# setup(
#     name=package_name,
#     version='0.0.0',  
#     packages=find_packages(exclude=['test']),
#     data_files= data_files,
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='Lakshya Agarwal',
#     maintainer_email='100lakshyaagarwal@gmail.com',
#     description='Ignition Fortress + TurtleBot3 bringup for ROS2 Humble',
#     license='TODO: License declaration',
#     tests_require=['pytest'],
#     entry_points={
#         'console_scripts': [

#         ],
#     },
# )

from setuptools import setup
import os
from glob import glob

package_name = 'tb3_fortress_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Include launch files
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),

        # Include world files
        (os.path.join('share', package_name, 'worlds'), 
         glob('worlds/*.sdf')),

        # Include config files
        (os.path.join('share', package_name, 'config'), 
         glob('config/*.yaml')),

        # Include model files - recursive for subdirectories
        (os.path.join('share', package_name, 'models', 'turtlebot3_burger'), 
         glob('models/turtlebot3_burger/*')),

        # Include scripts
        (os.path.join('share', package_name, 'scripts'), 
         glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='TurtleBot3 Gazebo Fortress simulation package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)