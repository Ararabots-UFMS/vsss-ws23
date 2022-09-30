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
        Returns if topic is already live.
        :param topic: String
        :return: bool
        """
        
        for node_name, namespace in node.get_node_names_and_namespaces():
            for topic_name,_ in node.get_publisher_names_and_types_by_node(node_name, namespace):
                if topic_name == topic:
                        return True

        return False


    @staticmethod
    def number_of_node_instances(node_name: str, node: Node):
        """
        Returns the number of node instances.
        :param topic: String
        :return: int
        """
        count = 0

        for current_node_name, _ in node.get_node_names_and_namespaces():
            if node_name == current_node_name:
                count += 1

        return count

    @staticmethod
    def number_of_topic_instances(topic_name: str, node: Node):
        """
        Returns the number of instances of a topic.
        e.g: game_topic returns 3 when:
            - game_topic_0
            - game_topic_1
            - game_topic_2
        :param topic: String
        :return: int
        """
        count = 0

        for topic_struct in node.get_topic_names_and_types():
            _, topic_type = topic_struct
            if topic_type == [topic_name]:
                count += 1

        return count #random.randint(0,99999)
