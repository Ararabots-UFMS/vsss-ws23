from typing import List
from urllib import request
import rclpy
from rclpy.node import Node
from sys_interfaces.msg import GameTopic
from sys_interfaces.srv import VisionCommand, MessageServerService
from message_server.opcodes import ServerOpCode
import numpy as np
import platform
from rclpy.qos import QoSPresetProfiles
class GameTopicPublisher:
    """
    This class can publish Game related messages on a 'Game topic' Topic
    :return: nothing
    """

    def __init__(self, node: Node = False, 
                       _game_opt: dict = None, 
                       _robot_params: dict = None,
                       _robot_name_roles: dict = None, 
                       owner_id = 'Player_One'):
        """
        :param _game_opt: Game json
        :param _robot_params: Robots Json
        :param _robot_name_roles: Robot roles Json
        :param owner_id: int
        """
        
        self._node = node

        # else is only a publisher
        self.name = 'game_topic'
        self.owner_id = str(owner_id)
        self.game_topic_name = np.zeros(256, dtype=np.uint8)
        #TODO: Talvez nao seja a melhor maneira, revisar
        for c_index, character in enumerate(list((self._node.get_namespace()+'\0').encode())):
            self.game_topic_name[c_index] = character

        self.pub = node.create_publisher(GameTopic, self.name, qos_profile=QoSPresetProfiles.SYSTEM_DEFAULT.value)
        self.msg = GameTopic()
        self.msg.robot_roles = [0, 0, 0, 0, 0]
        self.msg.robot_tags = [0, 0, 0, 0, 0]
        self.game_opt = _game_opt
        self.robot_params = _robot_params
        self.robot_name_roles = _robot_name_roles

        # Init message values
        self.faster_hash = ['robot_' + str(x) for x in range(1, 6)]

        for robot_id in range(5):
            role_name = self.robot_params[self.faster_hash[robot_id]]['role']
            self.set_robot_role(robot_id, self.robot_name_roles[role_name])
            self.set_robot_tag(robot_id, self.robot_params[self.faster_hash[robot_id]]['tag_number'])

        self.set_freeball_robot(self.game_opt['freeball_player'])
        self.set_penalty_robot(self.game_opt['penalty_player'])
        self.set_meta_robot(self.game_opt['meta_player'])

        self.set_team_side(self.game_opt['side'])
        self.set_team_color(self.game_opt['time'])

        # Variable for storing proxy
        self.vision_proxy = None
        self._messageserver_proxy = None
        self.message_server_name = None

        self.register_vision_service()
        self._register_messageserver_service(self.owner_id)

    def set_game_state(self, _game_state):
        """
        Sets the running game state
        :param _game_state: int
        :return: nothing
        """
        self.msg.game_state = _game_state

    def get_name(self):
        """
        Returns the name of game topic  
        :return: nothing
        """
        return self.name

    def get_owner(self):
        return self.owner_id

    def set_robot_role(self, robot_id, role):
        """
        Given a robot id, set its role in the game
        :param robot_id: int
        :param role: int
        :return: nothing
        """
        self.msg.robot_roles[robot_id] = role

    def set_robot_tag(self, robot_id, tag):
        """
        Given a robot id, set its tag in the game
        :param robot_id: int
        :param tag: int
        :return: nothing
        """
        self.msg.robot_tags[robot_id] = tag

    def assigned_action_to_robot(self, action_id, robot_id):
        """
        Assigned a robot_id to action(meta, free or penalty)
            - 0 -> Freeball
            - 1 -> Penalti
            - 2 -> Tiro de meta
        :param action_id: int
        :param robot_id: int
        :return: nothing
        """
        if action_id:
            if action_id == 1:
                self.set_penalty_robot(robot_id)
            else:
                self.set_meta_robot(robot_id)
        else:
            self.set_freeball_robot(robot_id)

    def set_penalty_robot(self, _penalty_robot):
        """
        Set the selected robot the action of taking penalty actions
        :param _penalty_robot: int
        :return: nothing
        """
        self.msg.penalty_robot = _penalty_robot

    def set_freeball_robot(self, _freeball_robot):
        self.msg.freeball_robot = _freeball_robot

    def set_meta_robot(self, _meta_robot):
        """
        Set the selected robot the action of taking meta-shot actions
        :param _meta_robot: int
        :return: nothing
        """
        self.msg.meta_robot = _meta_robot

    def set_message(self, game_state, 
                          team_side, 
                          team_color, 
                          robot_roles, 
                          robot_tags, 
                          penalty_robot, 
                          freeball_robot, 
                          meta_robot):
                          
        self.msg = GameTopic(
            game_state,
            team_side,
            team_color,
            tuple(robot_roles),
            tuple(robot_tags),
            penalty_robot,
            freeball_robot,
            meta_robot
        )

    def set_team_side(self, side):
        """
        Set side of the game, right(1) or left(0)?
        :param side: int
        :return: nothing
        """
        self.msg.team_side = side

    def set_team_color(self, color: int) -> None:
        """ Set side of the game, yellow(1) or blue(0)? """
        self.msg.team_color = color

    def publish(self):
        """
        This function publishes in the game topic
        :return: nothing
        """
        try:
            self.pub.publish(self.msg)
        except Exception as exception:
            self._node.get_logger().fatal(exception)

    def _register_messageserver_service(self, owner_id=None):
        self.message_server_name = '/'+platform.node().replace('-','_')+'/message_server_service'
        # wait_for_service(self.message_server_name)
        self._messageserver_proxy = self._node.create_client(MessageServerService,
                                                self.message_server_name)

    def add_or_remove_socket_on_messageserver(self, opcode: int,
                                              socket_id: int,
                                              socket_offset: int,
                                              mac_addr: List[int]) -> int:
        # wait_for_service(self.message_server_name)
        try:
            request = MessageServerService.Request()
            
            request.opcode = opcode
            request.socket_id = socket_id
            request.socket_offset = socket_offset
            request.game_topic_name = self.game_topic_name
            #TODO: Arrumar isso aqui, ta feio (Bluetooth)
            request.robot_mac_addr[0] = mac_addr[0]
            request.robot_mac_addr[1] = mac_addr[1]
            request.robot_mac_addr[2] = mac_addr[2]
            request.robot_mac_addr[3] = mac_addr[3]
            request.robot_mac_addr[4] = mac_addr[4]
            request.robot_mac_addr[5] = mac_addr[5]
            
            r = self._messageserver_proxy.call(request)
            return r.response
        except Exception as exception:
            self._node.get_logger().fatal(str(type(mac_addr[0]))+" - "+str(type(socket_id)))
            self._node.get_logger().fatal("Message server service request error " + repr(exception) + repr(request))
            return ServerOpCode.ERROR.value

    def register_vision_service(self):
        """
        Creates a proxy for communicating with service
        :return: nothing
        """
        # wait_for_service('vision_command')
        self.vision_proxy = self._node.create_client(VisionCommand, '/vision_command')

    def send_vision_operation(self, operation):
        """
        Sends a service request to vision node
        :param operation : uint8
        :return: nothing
        """
        self.vision_proxy.wait_for_service()
        try:
            request = VisionCommand.Request()
            request.operation = operation
            self.vision_proxy.call(request)
        except Exception as exception:
            self._node.get_logger().fatal("Service did not process request: " + str(exception))


if __name__ == '__main__':
    try:
        GameTopicPublisher()
    except Exception as exception:
        pass
