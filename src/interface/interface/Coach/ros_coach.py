from typing import Dict
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from robot.robot import Robot

class RosCoach:
    """
    This class is responsible for creating, modifying and launch robot nodes
    """
    def __init__(self, node: Node):
        """
        :param node: Node
        :return: nothing
        """
        
        self._node = node
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)
        
        # Allocate robots process
        self.player_nodes = {}
        self.player_params = {}

    def create_and_store_node(self, robot_index: int, robot_name: str, active: bool, variables: Dict):
        """
        Creates a robot node and stores its process for later use
        :param robot: String
        :param robot_name: String
        :param active: Bool
        :param variables: Dict
        :return: nothing
        """

        
        self.player_params[robot_name] = variables

        if int(active):
            node = self.create_node(robot_name)
            # Lets store the node for future alterations
            self.player_nodes[robot_name] = node
            self._executor.add_node(node)

    def create_node(self, robot_name: str) -> Node:
        # creates a node with robot list arguments

        variables = self.player_params[robot_name]

        return Robot(
            robot_id=variables["robot_id"], 
            tag=variables["tag_number"], 
            robot_body=variables["body_id"], 
            team_side=variables["team_side"], 
            team_color=variables["team_color"], 
            robot_role=variables["role"], 
            owner_name=self._node.get_namespace(), 
            socket_id=variables["socket_id"], 
            should_debug=variables["should_debug"]
            )
            
    def stop_node(self, robot_name: str):
        self._node.get_logger().fatal("Called destructor")
        robot_node = self.player_nodes[robot_name]
        try:
            self._executor.remove_node(robot_node)
            robot_node.destroy_node()
        except Exception as exception:
            self._node.get_logger().fatal(repr(exception))

    def change_arguments_of_node(self, robot_name: str, variables: Dict):
        """
        Change arguments from a node, restart its process in case it is already running
        :param robot_name: String
        :param variables: Dict
        :return: nothing
        """
        self.player_params[robot_name] = variables
        
        if (robot_name in self.player_nodes.keys()) and self.player_nodes[robot_name] in self._executor.get_nodes():
            # We need to restart the node :<
            self.stop_node(robot_name)
            self.player_nodes[robot_name] = self.create_node(robot_name)

    def spin_function(self, func_to_spin):
        future_obj = self._executor.create_task(func_to_spin)
        self._executor.spin_until_future_complete(future_obj)

    def toggle_node_life(self, robot_name, should_be_active):
        """
        Given a robot name, toggle its life
        :param robot_name: String
        :param should_be_active: Bool
        :return: nothing
        """
        node_created = (robot_name in self.player_nodes.keys()) and (self.player_nodes[robot_name] is not None)
        active = node_created and (self.player_nodes[robot_name] in self._executor.get_nodes())
        choice = 0
        if should_be_active:  # This robot should be active
            
            if not node_created:
                choice = 1
                node = self.create_node(robot_name)
                self.player_nodes[robot_name] = node
                self._executor.add_node(node)
            elif not active:  # Missing the process
                choice = 2
                self._executor.add_node(self.player_nodes[robot_name])
            else:  # Do nothing if process is ok
                choice = 3
                pass
        else:  # I dont want you anymore
            
            if active:
                choice = -1
                self.stop_node(robot_name)            
            elif node_created:
                choice = -2 
                try:
                    self.player_nodes[robot_name].destroy_node()
                except Exception as exception:
                    self._node.get_logger().fatal(repr(exception))
            else:
                choice = -3
            self.player_nodes[robot_name] = None
        self._node.get_logger().fatal("Choice "+str(choice))

