import socket
import struct
from time import time
import referee.vssref_command_pb2 as command
import referee.vssref_common_pb2 as common


import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from utils.ros_utils import MsgOrigin
from interface.ros_game_topic_publisher import GameTopicPublisher

from utils.model import Model

from random import randint

from sys import argv

# from referee.replacer import ReplacerInterface


class RefereeNode(Node):

    ref_to_game_state = [
        2, # Free ball
        3, # Penalty
        4, # Tiro de meta
        2, # Free ball
        1, # Jogo normal -> NO FUTURO USAR REPLACE
        0, # Stop 
        1, # Jogo normal
        0  # Stop
    ]

    def __init__(self, owner_id: str, team_side: int = None,  team_color: int = None, referee_ip = "224.5.23.2", referee_port = 10003):
        super().__init__('referee')
        self.referee_ip = referee_ip
        self.referee_port = referee_port

        self.referee_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.referee_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        self.referee_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 128)
        self.referee_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
        self.referee_sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, struct.pack("=4sl", socket.inet_aton(self.referee_ip), socket.INADDR_ANY))
        self.referee_sock.bind((self.referee_ip, self.referee_port))

        model = Model()
        self.game_topic_pub = GameTopicPublisher(None, model.game_opt, model.robot_params, model.robot_roles, owner_id=owner_id)
        self.game_topic_pub.msg.msg_origin = MsgOrigin.REFEREE.value
        self.game_topic_pub.msg.automatic_position = 1

        self.message = command.VSSRef_Command()

        # # Estes parâmetros são None quando a interface é utilizada (sim_main.launch)
        # if team_side is not None and team_color is not None:
        #     self.replacer = ReplacerInterface(
        #         team_side,
        #         team_color 
        #     )
        # else:
        #     # Usa-se os parâmetros do game-topic
        #     self.replacer = ReplacerInterface(
        #         self.game_topic_pub.msg.team_side,
        #         self.game_topic_pub.msg.team_color 
        #     )

        # Armazenando o último estado de jogo.
        self._last_game_event = 5

    def _publish_event(self, event: int) -> None:

        # TODO: Atualmente, dois modos: STOPPED e GAME_ON
        # Os estados de PENALTY e FREEBALL nao estão sendo usados, pois há transições nos pacotes que o juiz manda
        # Ex: Penalty ocorreu:
        # Pacotes enviados pelo juiz: Penalty -> Stopped -> Game on
        
        if event <= 4:
            self.game_topic_pub.set_game_state(0)
            self._last_game_event = event
        else:
            if event == 6 and self._last_game_event <= 4:
                event = self._last_game_event
                print(event)

            state = RefereeNode.ref_to_game_state[event]
            self.game_topic_pub.set_game_state(state)
            
            self.game_topic_pub.publish()

    def tick(self):

        data, _ = self.referee_sock.recvfrom(1024) # buffer size is 1024 bytes
        self.message.ParseFromString(data)

        event = self.message.foul

        # state = RefereeNode.ref_to_game_state[event]

        # self.game_topic_pub.set_game_state(state)
        # self.game_topic_pub.publish()

        # TODO: Se o lado do time mudar -> PIPOCO
        # TODO: Este no tem que escutar o game topic, para saber o lado atual...
        # TODO: bolar uma lógica para preparar o próximo game_state. Depois que uma falta ocorre, há período de espera.

        # Caso o estado do jogo tenha sido alterado, atualizar o game_topic
        # rospy.logdebug(f"Novo evento {event}")

        # Remover este if no futuro
        # if event < 5:
        #     # Informações de interesse. Acho que nao precisamos saber em que tempo do jogo estamos 
        #     # nem a timestamp do evento...
        #     # rospy.logfatal(f"- Tipo (foul): {self.message.foul}")
        #     # rospy.logfatal(f"- Cor do time: {self.message.teamcolor}")
        #     # rospy.logfatal(f"- Quadrante: {self.message.foulQuadrant}")
        #     # Interromper o jogo neste caso  
        #     self.replacer.handle_event(self.message)

        self._publish_event(event)

        print(event)

        
def main(args=None):
    rclpy.init(args=args)

    referee = RefereeNode(owner_id='vsss')
    while 1:
        referee.tick()

if __name__ == '__main__':
    main()