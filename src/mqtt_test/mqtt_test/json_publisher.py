#!/usr/bin/env python3
"""
:module:        json_publisher.py
:description:   ROS2 node that publishes JSON messages to an MQTT broker.
:owner:         (C) C-Boost B.V. (cboost) - All Rights Reserved
:author:        [Patrick Verwimp](mailto:patrick.verwimp@cboost.nl)
:project:       Cboost Internal

This file is proprietary and confidential.
Unauthorized copying of this file via any medium is strictly prohibited.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class JsonPublisher(Node):
    """
    A ROS2 node that publishes JSON messages to an MQTT broker.
    """
    
    def __init__(self):
        super().__init__('json_publisher')
        self._declare_parameters()        
        self.publisher = self.create_publisher(
            String, 
            self.get_parameter('topic_name').get_parameter_value().string_value,
            10
        )
        self.timer = self.create_timer(
            self.get_parameter('publish_rate').get_parameter_value().double_value,
            self.publish_json_message
        )

    def _declare_parameters(self):
        """
        Declare parameters for the node.
        """
        self.declare_parameter('topic_name', 'mqtt_out')
        self.declare_parameter('publish_rate', 1.0)

    def publish_json_message(self):
        """Publish a JSON message."""
        data = {
            'name': 'Cboost',
            'type': 'ROS2',
            'version': '1.0'
        }
        json_data = json.dumps(data)
        msg = String(data=json_data)
        self.publisher.publish(msg)
        self.get_logger().info(f'Published JSON: {json_data}')


def main(args=None):
    rclpy.init(args=args)
    node = JsonPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()