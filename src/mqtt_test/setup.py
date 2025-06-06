from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'mqtt_test'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Patrick Verwimp',
    description='A package for testing MQTT communication in ROS 2',
    license='TODO: License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'json_publisher = mqtt_test.json_publisher:main',
            'mqtt_subscriber = mqtt_test.mqtt_subscriber:main',
        ],
    },
)

