from typing import List
import numpy as np
from rclpy.node import Node
from sys_interfaces.msg import MessageServerTopic
import platform

class SenderPublisher:
    def __init__(self, node: Node, socket_id: int, socket_offset: int):
        self.TAG = "ROBOT SENDER PUBLISHER"
        self._node = node
        user_namespace = platform.node().replace('-','_')
        self.publisher = self._node.create_publisher(
                            MessageServerTopic,
                            '/'+user_namespace+'/message_server_topic',
                            qos_profile=1)
        self.msg = MessageServerTopic()
        self.msg.socket_id = socket_id
        self.msg.socket_offset = socket_offset

    def publish(self, priority: int, socket_id: int, msg: List):
        # self.msg.socket_id = socket_id
        
        self.msg.priority = priority
        try:
            #TODO: melhorar conversao de lista para np array
            self.msg.payload[0] = msg[0]
            self.msg.payload[1] = msg[1]
            self.msg.payload[2] = msg[2]
            self.publisher.publish(self.msg)
        except Exception as e:
            self._node.get_logger().fatal(self.TAG + " " + str(socket_id) + ": UNABLE TO PUBLISH MESSAGE: "
                           + repr(msg) + " " + repr(e))
