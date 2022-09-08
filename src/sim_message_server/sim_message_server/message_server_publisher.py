from typing import List
import numpy as np

import rclpy
from rclpy.node import Node
from sys_interfaces.msg import ConnectionStatusTopic


class MessageServerPublisher:
    def __init__(self, node: Node, owner_id: str = None):
        self.TAG = "MESSAGE SERVER PUBLISHER"
        self._node = node
        self._msg = ConnectionStatusTopic()        
        self.publisher = self._node.create_publisher(
                            ConnectionStatusTopic,
                            'connection_status_topic',
                            qos_profile=10)

    def publish(self, sockets_status: np.array) -> None:
        self._msg.sockets_status = sockets_status
        try:
            self.publisher.publish(self._msg)
        except Exception as exception:
            self._node.get_logger().fatal(self.TAG + ": UNABLE TO PUBLISH. " + repr(exception) + repr(self._msg))
            raise exception
