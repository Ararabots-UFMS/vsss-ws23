from logging import debug
from typing import List, NewType
from enum import Enum
from time import sleep
from rclpy.node import Node
from collections import namedtuple
from utils.socket_interfaces import SenderSocket
import numpy as np
from sys_interfaces.srv import MessageServerService
from message_server.opcodes import ServerOpCode
from sys_interfaces.msg import MessageServerTopic, GameTopic
from sim_message_server.message_server_publisher import MessageServerPublisher

Message = namedtuple("Message", ["priority", "counter", "socket_id", "payload"])

Seconds = NewType('seconds', float)


import sim_cam.sim.packet_pb2 as packet_pb2
import sim_cam.sim.command_pb2 as command_pb2


class MessageServer:

    UDP_IP = ""
    UDP_PORT = 20011

    def __init__(self, node: Node,
                 owner_id: str = None, 
                 team_color: int = 0,
                 max_sockets_capacity: int = 5,
                 max_queue_size: int = 5,
                 socket_timeout: Seconds = 0.00064
                 ):

        self._node = node
        self._simulator_multiplyer = 80/255
        self._capacity = max_sockets_capacity
        self.socket_timeout = socket_timeout

        self.TAG = "MESSAGE SERVER"

        self.game_topic_subs = {}

        self._sockets_status_matrix = [np.zeros(self._capacity, dtype= np.uint8) for _ in range(self._capacity)]

        self.topic_publisher = MessageServerPublisher(self._node, self._sockets_status_matrix, self._capacity)

        self.service = self._node.create_service(MessageServerService,
                                    'message_server_service',                                     
                                     self._service_request_handler)

        self.topic_publisher.publish_all()

        # ============================================

        self.sock = SenderSocket.create(self.UDP_IP, self.UDP_PORT)
        
        self._socket_message_array = []
        self._cmd_array = []

        for _ in range(self._capacity):
            # Pode ser um singleton mesmo?            
            message = packet_pb2.Packet()        
            cmd = message.cmd.robot_commands.add()
            cmd.yellowteam = team_color
            self._socket_message_array.append(message)
            self._cmd_array.append(cmd)


    
        self._node.create_subscription(
                         MessageServerTopic,
                         'message_server_topic',
                         self._read_topic,
                         qos_profile=5)

    def create_game_topic_subscription(self, game_topic_name: str, socket_offset: int) -> None:
        if game_topic_name in self.game_topic_subs.keys():
            return

        self.game_topic_subs[game_topic_name] = self._node.create_subscription(
            GameTopic,
            game_topic_name+'/game_topic',
            lambda data: self._read_game_topic(socket_offset, data),
            qos_profile=5)

    def _read_topic(self, data: MessageServerTopic) -> None:
        self._sim_send(data.socket_id, data.socket_offset, data.payload)

    def _read_game_topic(self, socket_offset:int, data: GameTopic) -> None:
        self._cmd_array[socket_offset].yellowteam = data.team_color

    def _service_request_handler(self,
        request: MessageServerService.Request,
        response: MessageServerService.Response) -> int:

        response_value = ServerOpCode.ERROR
        game_topic_name = ''.join([chr(i) for i in request.game_topic_name]).strip('\0')
        self.create_game_topic_subscription(game_topic_name, request.socket_offset)

        if request.opcode == ServerOpCode.ADD.value:
            response_value = self._add_socket(request.socket_id, request.socket_offset)

        elif request.opcode == ServerOpCode.REMOVE.value:
            response_value = self._remove_socket(request.socket_id, request.socket_offset)

        self.topic_publisher.publish(request.socket_offset)
        
        sleep(2)

        response.response = response_value.value

        return response

    def _update_socket_status(self, sock_id: int, socket_offset:int, status: Enum) -> None:

        if status == ServerOpCode.ACTIVE:
            self._sockets_status_matrix[socket_offset][sock_id] = 1
        else:
            self._sockets_status_matrix[socket_offset][sock_id] = 0

    def _add_socket(self, socket_id: int, socket_offset:int) -> ServerOpCode:       
        self._update_socket_status(socket_id, socket_offset, ServerOpCode.ACTIVE)
        return ServerOpCode.OK

    def _remove_socket(self, socket_id: int, socket_offset:int) -> ServerOpCode:
        self._update_socket_status(socket_id, socket_offset, ServerOpCode.INACTIVE)
        return ServerOpCode.OK

    def on_shutdown(self) -> None:
        self.sock.close()

    def _sim_send(self, id_: int, socket_offset: int, payload: List):
        self._cmd_array[socket_offset].id = id_        
        # self.LEFTFORWARD_RIGHTFORWARD = 0x00  # 0000 0000
        # self.LEFTFORWARD_RIGHTBACKWARD = 0x01  # 0000 0001
        # self.LEFTBACKWARD_RIGHTFORWARD = 0x02  # 0000 0010
        # self.LEFTBACKWARD_RIGHTBACKWARD = 0x03  # 0000 0011
    
        # O payload passou a ser um vetor float32[3] para simulação.
        # Por isso, é necessário fazer um cast para int na primeira posição antes de fazer a operação de bits
        self._cmd_array[socket_offset].wheel_left =  (-payload[1] if (int(payload[0])&2) else payload[1]) * self._simulator_multiplyer 
        self._cmd_array[socket_offset].wheel_right = (-payload[2] if (int(payload[0])&1) else payload[2]) * self._simulator_multiplyer
        
        # self._node.get_logger().fatal(f"{id_} {repr(payload)}")
        payload = self._socket_message_array[socket_offset].SerializeToString()        
        self.sock.sendall(payload)