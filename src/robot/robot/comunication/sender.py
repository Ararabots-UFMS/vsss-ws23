from rclpy.node import Node
from typing import Union, List
from robot.comunication.sender_publisher import SenderPublisher
from collections import namedtuple

STDMsg = namedtuple("STDMsg", ["left_speed", "right_speed"])
SelfControlMsg = namedtuple("SelfControlMsg", ["direction", "speed", "delta_theta"])


class Sender:
    def __init__(self, node: Node, socket_id: int,socket_offset: int, owner_name: str = None):
        self._socket_id = socket_id
        self.publisher = SenderPublisher(node, socket_id, socket_offset)

    def send(self, priority: int,
             msg: List) -> None:
        self.publisher.publish(priority, self._socket_id, msg)
