from rclpy.node import Node
import random

from enum import Enum

class MsgOrigin(Enum):
    GAME_TOPIC      =   0
    REFEREE         =   1
    TRAINER         =   2

class RosUtils:
    def __init__(self):
        pass

    @staticmethod
    def topic_exists(topic: str, node: Node):
        """
        Returns if topic is already live
        :param topic: String
        :return: bool
        """
        
        for node_name, namespace in node.get_node_names_and_namespaces():
            for topic_name,_ in node.get_publisher_names_and_types_by_node(node_name, namespace):
                if topic_name == topic:
                        return True

        return False

    @staticmethod
    def number_of_topic_instances(topic):
        """
        Returns the number of instances of a topic, given a prefix
        e.g: game_topic returns 3 when:
            - game_topic_0
            - game_topic_1
            - game_topic_2
        :param topic: String
        :return: int
        """        
        return random.randint(0,99999)
