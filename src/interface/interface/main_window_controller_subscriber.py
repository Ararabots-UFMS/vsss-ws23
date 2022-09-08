from typing import Callable, List
import rclpy
from rclpy.node import Node
from sys_interfaces.msg import ConnectionStatusTopic

Callback = Callable[[List], None]


class MainWindowControllerSubscriber:
    def __init__(self, node:Node, callback: Callback, owner_id: str = None):
        self._callback = callback
        node.create_subscription(
                        ConnectionStatusTopic,
                        'connection_status_topic',
                        self._read_topic,
                        qos_profile=5)

    def _read_topic(self, data: ConnectionStatusTopic) -> None:
        self._callback(data.sockets_status)
