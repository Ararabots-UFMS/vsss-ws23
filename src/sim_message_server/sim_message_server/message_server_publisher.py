from typing import List
import numpy as np

import rclpy
from rclpy.node import Node
from sys_interfaces.msg import ConnectionStatusTopic


class MessageServerPublisher:
    def __init__(self, node: Node, owner_id: str = None):
        self.TAG = "MESSAGE SERVER PUBLISHER"
        self._node = node
        suffix = '' if owner_id is None else '_' + owner_id
        self.publisher = self._node.create_publisher(
                            ConnectionStatusTopic,
                            'connection_status_topic' + suffix,
                            qos_profile=10)

    def publish(self, sockets_status: List) -> None:
        msg = sockets_status
        try:
            self.publisher.publish(msg)
        except Exception as exception:
            self._node.get_logger().fatal(self.TAG + ": UNABLE TO PUBLISH. " + repr(exception))
            raise exception
