from typing import List
from rclpy.node import Node
import numpy as np

from sys_interfaces.msg import ConnectionStatusTopic


class MessageServerPublisher:
    def __init__(self, node: Node, sockets_status_matrix = None, capacity: int = 5):
        self.TAG = "MESSAGE SERVER PUBLISHER"
        self._node = node
        self._capacity = capacity
        self._msg_matrix = [ConnectionStatusTopic() for _ in range(self._capacity)]
        self._sockets_status_matrix = sockets_status_matrix
        self.publisher = self._node.create_publisher(
            ConnectionStatusTopic,
            'connection_status_topic',
            qos_profile=10)

    def publish_all(self):
        for socket_msg in self._msg_matrix:
            try:
                self.publisher.publish(socket_msg)
            except Exception as exception:
                self._node.get_logger().fatal(self.TAG + ": UNABLE TO PUBLISH. " + repr(exception) + repr(socket_msg))
                raise exception

    def publish(self, socket_offset: int) -> None:
        socket_msg = self._msg_matrix[socket_offset]
        socket_msg.sockets_status = self._sockets_status_matrix[socket_offset]
        try:
            self.publisher.publish(socket_msg)
        except Exception as exception:
            self._node.get_logger().fatal(self.TAG + ": UNABLE TO PUBLISH. " + repr(exception) + repr(socket_msg))
            raise exception
