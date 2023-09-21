#!/usr/bin/python3
from re import T
from types import new_class
import referee.sim.vssref_command_pb2 as command_pb2
from referee.sim.vssref_common_pb2 import Frame
import referee.sim.vssref_placement_pb2 as placement_pb2

import sys
import cv2
from random import randint
from sys import argv

from utils.model import Model
import rospy

import os
from enum import Enum
old_path = sys.path[0]
sys.path[0] = root_path = os.environ['ROS_ARARA_ROOT']+"src/"
from ROS.ros_game_topic_publisher import GameTopicPublisher 
from ROS.ros_vision_publisher import RosVisionService
from ROS.ros_utils import MsgOrigin, RosUtils
sys.path[0] = old_path
from utils.socket_interfaces import ReceiverSocket

from verysmall.msg import game_topic

from replacer import ReplacerInterface

class RefereeNode:

    """
    A node for spinning the Vision
    """

    '''
    FREE_KICK = 0;
	PENALTY_KICK = 1;
	GOAL_KICK = 2;
	FREE_BALL = 3;
	KICKOFF = 4;
	STOP = 5;
	GAME_ON = 6;
	HALT = 7;
    '''

    ref_to_game_state = [
        2, # Free ball
        3, # Penalty
        4, # Tiro de meta
        2, # Free ball
        0, # Jogo normal -> NO FUTURO USAR REPLACE
        0, # Stop 
        1, # Jogo normal
        0  # Stop
    ]

    UDP_IP = "224.5.23.2"
    UDP_PORT = 10003

    def __init__(self, owner_id: str, team_side: int = None,  team_color: int = None):

        model = Model()
        
        if team_side is not None and team_color is not None:
            model.game_opt['side'] = team_side
            model.game_opt['time'] = team_color

        self.mercury = GameTopicPublisher(True, model.game_opt, model.robot_params, model.robot_roles, owner_id)        
        self.mercury.msg.msg_origin = MsgOrigin.REFEREE.value

        self.message = command_pb2.VSSRef_Command() # singleton para comandos de leitura, só precisamos parsear (que palavra feia KK) dados que chegam.

        # Estes parâmetros são None quando a interface é utilizada (sim_main.launch)
        if team_side is not None and team_color is not None:
            self.replacer = ReplacerInterface(
                team_side,
                team_color 
            )
        else:
            # Usa-se os parâmetros do game-topic
            self.replacer = ReplacerInterface(
                self.mercury.msg.team_side,
                self.mercury.msg.team_color 
            )

        # Configuração dos sockets
        self.sock = ReceiverSocket.create(self.UDP_IP, self.UDP_PORT) 
        # Armazenando o último estado de jogo.
        self._last_game_event = 5

    def _publish_event(self, event: int) -> None:

        # TODO: Atualmente, dois modos: STOPPED e GAME_ON
        # Os estados de PENALTY e FREEBALL nao estão sendo usados, pois há transições nos pacotes que o juiz manda
        # Ex: Penalty ocorreu:
        # Pacotes enviados pelo juiz: Penalty -> Stopped -> Game on
        
        if event <= 4:
            self.mercury.set_game_state(0)
            self._last_game_event = event
        else:
            if event == 6 and self._last_game_event <= 4:
                event = self._last_game_event

            state = RefereeNode.ref_to_game_state[event]
            self.mercury.set_game_state(state)





        # if event < 4:
        #     self.mercury.set_game_state(0)
        #     self._last_game_event = event
        # else:
        #     if (event == 6) and (self._last_game_event < 4):
        #         event = self._last_game_event
        #         self._last_game_event = event
            
        #     state = RefereeNode.ref_to_game_state[event]
        #     self.mercury.set_game_state(state)
        
        # rospy.logfatal(event)
        self.mercury.publish()

    def tick(self):

        data, _ = self.sock.recvfrom(1024) # buffer size is 1024 bytes
        self.message.ParseFromString(data)

        event = self.message.foul

        state = RefereeNode.ref_to_game_state[event]

        self.mercury.set_game_state(state)
        self.mercury.publish()

        # TODO: Se o lado do time mudar -> PIPOCO
        # TODO: Este no tem que escutar o game topic, para saber o lado atual...
        # TODO: bolar uma lógica para preparar o próximo game_state. Depois que uma falta ocorre, há período de espera.

        # Caso o estado do jogo tenha sido alterado, atualizar o game_topic
        rospy.logdebug(f"Novo evento {event}")

        # Remover este if no futuro
        if event < 5:
            # Informações de interesse. Acho que nao precisamos saber em que tempo do jogo estamos 
            # nem a timestamp do evento...
            rospy.logfatal(f"- Tipo (foul): {self.message.foul}")
            rospy.logfatal(f"- Cor do time: {self.message.teamcolor}")
            rospy.logfatal(f"- Quadrante: {self.message.foulQuadrant}")
            # Interromper o jogo neste caso
            self.replacer.handle_event(self.message)

        self._publish_event(event)

    def vision_management(self, req):
        """
        This is the reading function for a service response
        :param req: variable to get the request operation
        :return: bool
        """
        success = True
        self.state_changed = req.operation
        return success


if __name__ == "__main__":
    try:
        owner_id = argv[1]
    except ValueError:
        owner_id = 'Player_' + str(randint(0, 99999))

    try:
        vision_node = RefereeNode(owner_id, int(argv[2]), int(argv[3]))
    except:
        vision_node = RefereeNode(owner_id)

    # rate = rospy.Rate(30)  # 30hz

    while not rospy.is_shutdown():
        vision_node.tick()
        # rate.sleep()