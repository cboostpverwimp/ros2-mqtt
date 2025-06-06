#!/usr/bin/env python3
"""
:module:        publish_test.launch.py
:description:   Launch MQTT client and JSON publisher nodes for testing MQTT communication.
:owner:         (C) C-Boost B.V. (cboost) - All Rights Reserved
:author:        [Patrick Verwimp](mailto:patrick.verwimp@cboost.nl)
:project:       Cboost Internal

This file is proprietary and confidential.
Unauthorized copying of this file via any medium is strictly prohibited.
"""

import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

__author__ = "Patrick Verwimp"
__copyright__ = "(C) C-Boost B.V. (cboost) - All Rights Reserved"
__credits__ = ["Patrick Verwimp", "Simon Mingaars"]
__license__ = "GPL"
__version__ = "0.0.1"
__maintainer__ = "Patrick Verwimp"
__email__ = "patrick.verwimp@cboost.nl"
__status__ = "Prototype"

def generate_launch_description():
    
    config = os.path.join(
        get_package_share_directory('mqtt_test'),
        'config',
        'mqtt_params.yaml'
    )

    mqtt_client = Node(
        package='mqtt_client',
        executable='mqtt_client',
        name='mqtt_client',
        output='screen',
        parameters=[ config ],
    )

    publisher = Node(
        package='mqtt_test',
        executable='json_publisher',
        name='json_publisher',
        output='screen',
        parameters=[
            {
                'publish_rate': 1.0,
                'topic_name': 'mqtt_out'
            }
        ],
    )

    return LaunchDescription(
        [
            mqtt_client,
            publisher,
        ]
    )