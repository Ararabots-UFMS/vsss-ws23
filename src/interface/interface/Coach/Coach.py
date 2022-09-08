from typing import Dict
from robot.robot import Robot
from interface.Coach.ros_coach import RosCoach
from interface.ros_game_topic_publisher import GameTopicPublisher
from sys_interfaces.msg import GameTopic
from rclpy.node import Node

class Coach:
    """Creates the Coach"""

    def __init__(self, node:Node, model,
                 _game_topic_publisher: GameTopicPublisher):

        # Save the parameters for future use
        self.robot_params = model.robot_params
        self.robot_bluetooth = model.robot_bluetooth
        self.robot_roles = model.robot_roles
        self.game_opt = model.game_opt
        self.debug_params = model.debug_params
        self.game_topic_pub = _game_topic_publisher
        # Fast access array to use a dict as an simple array
        self.faster_hash = ['robot_' + str(x) for x in range(1, 6)]
        
        self._node = node
        self.ros_functions = RosCoach(self._node)

        self.create_robots()

    def create_robots(self):
        """
        Reading from database, creates nodes and store them in process
        :return: nothing
        """
        # Doing loops for creating the robot nodes
        for robot_index, robot in enumerate(self.robot_params.keys()):
            # arguments for the node
            try:
                debug = str(self.debug_params["things"][robot])
                variables = self.get_robot_params(robot,
                                                  self.robot_params[robot],
                                                  self.game_topic_pub.msg,
                                                  int(debug))
            except KeyError:
                variables = {}
                self.robot_params[robot]['active'] = False
            

            # Creates a player node
            self.ros_functions.create_and_store_node(robot_index, robot, self.robot_params[robot]['active'], variables)

    def get_robot_params(self, robot_name: str,
                         robot_params: Dict,
                         topic: GameTopic,
                         debug: int = 0) -> Dict:
        robot_id = int(robot_name.split('_')[1]) - 1
        robot_id = str(robot_id)

        if robot_params["bluetooth_mac_address"] == "nenhum":
            socket_id = -1
        else:
            socket_id = robot_id

        params = {}
        params["robot_id"] = int(robot_id)
        params["tag_number"] = int(robot_params["tag_number"])
        params["body_id"] = robot_params["body_id"]
        params["team_side"] = str(topic.team_side)
        params["team_color"] = str(topic.team_color)
        params["role"] = topic.robot_roles[int(robot_id)]
        params["owner_name"] = self.game_topic_pub.get_owner()
        params["socket_id"] = int(socket_id)
        params["should_debug"] = str(debug)

        return params

    def change_robot_role(self, robot_id, role):
        """
        This function changes the coach role sugestion and publishes through game_topic
        :param robot_id: int
        :param role: int
        :return: nothing
        """
        # TODO: alterar objeto Coach
        # TODO: alterar funcao para receber string e transformar em int para publicar
        self.game_topic_pub.set_robot_role(robot_id, role)
        self.game_topic_pub.publish()

    def change_robot_tag(self, robot_id, tag):
        """
        This function changes the robot tag sugestion and publishes through game_topic
        :param robot_id: int
        :param tag: int
        :return: nothing
        """
        # TODO: alterar objeto Coach
        # TODO: alterar funcao para receber string e transformar em int para publicar
        self.game_topic_pub.set_robot_tag(robot_id, tag)
        self.game_topic_pub.publish()

    def set_robot_parameters(self, robot_id):
        """
        With robot_id, sets all the argument for node
        :param robot_id: int
        :return: nothing
        """
        robot = self.faster_hash[robot_id]

        # arguments for the node
        debug = self.debug_params["things"][robot]
        variables = self.get_robot_params(robot,
                                          self.robot_params[robot],
                                          self.game_topic_pub.msg,
                                          debug)

        self.ros_functions.change_arguments_of_node(robot, variables)

    def set_robot_active(self, robot_id, should_be_active):
        """
        Sets a defined robot_id node to start or stop
        :param robot_id: int
        :param should_be_active: bool
        :return: nothing
        """
        robot_name = self.faster_hash[robot_id]
        self.ros_functions.toggle_node_life(robot_name, should_be_active)

    # TODO: Definir run aqui e importala de outro arquivo
