from multiprocessing.sharedctypes import Value
from typing import List, Union, NewType
# import bluetooth
# from bluetooth import BluetoothSocket, BluetoothError
from enum import Enum
from time import time, sleep
from collections import deque
from rclpy.node import Node
from collections import namedtuple
import heapq as hp
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from now_message_server.message_server_publisher import MessageServerPublisher
from now_message_server.opcodes import ServerOpCode
from sys_interfaces.msg import MessageServerTopic
from sys_interfaces.srv import MessageServerService

import numpy as np

Seconds = NewType('seconds', float)


class MessageServer:

    def __init__(self, node: Node,
                 owner_id: str = None, 
                 max_sockets_capacity: int = 5,
                 max_queue_size: int = 5,
                 socket_timeout: Seconds = 0.00064):

        self._node = node

        self._capacity = max_sockets_capacity
        self._max_queue_size = max_queue_size
        # self._node.get_logger().warning(f"{socket_id} - {self._capacity}")

            
        self.socket_timeout = socket_timeout

        self.TAG = "MESSAGE SERVER"

        self._num_active_sockets = 0
        self._sockets = []
        # The payload for esp now
        self._socket_message_matrix = [[[0,0,0,0,0] for _ in range(self._capacity)] for _ in range(self._capacity)]
        self._sockets_status_matrix = [np.zeros(self._capacity, dtype= np.uint8) for _ in range(self._capacity)] # [0] * self._capacity

        self.topic_publisher = MessageServerPublisher(self._node, self._sockets_status_matrix, self._capacity)

        self.service = self._node.create_service(MessageServerService,
                                    'message_server_service',                                     
                                     self._service_request_handler)

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        self._node.create_subscription(
                         MessageServerTopic,
                         'message_server_topic',
                         self._read_topic,
                         qos_profile=qos_profile)
        
        self.topic_publisher.publish_all()


    def _read_topic(self, data: MessageServerTopic) -> None:
        self._node.get_logger().warning(f"Writting at Serial:{data.socket_id}")
        socket_message = self._socket_message_matrix[data.socket_offset] 
        socket_message[data.socket_id][2] = data.payload[0]
        socket_message[data.socket_id][3] = data.payload[1]
        socket_message[data.socket_id][4] = data.payload[2]
        self._node.get_logger().warning(f"Writting at Serial:{socket_message[data.socket_id]}")


    def _service_request_handler(self,
                                 request, response) -> int:
        response_value = ServerOpCode.ERROR

        if request.opcode == ServerOpCode.ADD.value:
            response_value = self._add_socket(request.socket_id, request.socket_offset, request.robot_mac_addr)

        elif request.opcode == ServerOpCode.REMOVE.value:
            response_value = self._remove_socket(request.socket_id, request.socket_offset, request.robot_mac_addr)

        
        self.topic_publisher.publish(request.socket_offset)
        
        sleep(2)

        response.response = response_value.value

        return response


    def _mac_been_used(self, mac_address: bytes) -> bool:
        return ":".join("%02x" % b for b in mac_address) in self._sockets

    def _update_socket_status(self, sock_id: int, sock_offset: int, status: Enum) -> None:

        if status == ServerOpCode.ACTIVE:
            self._sockets_status_matrix[sock_offset][sock_id] = 1
        else:
            self._sockets_status_matrix[sock_offset][sock_id] = 0

    def _insert_mac_into_message(self, socket_id: int, sock_offset: int, mac_address: bytes) -> None:
        first_byte, second_byte = mac_address[-2:]

        self._socket_message_matrix[sock_offset][socket_id][0] = first_byte
        self._socket_message_matrix[sock_offset][socket_id][1] = second_byte

    def _add_socket(self, socket_id: int, sock_offset: int, mac_address: bytes) -> ServerOpCode:
        response = ServerOpCode.ERROR

        if self._num_active_sockets < self._capacity and not self._mac_been_used(mac_address):
            self._num_active_sockets += 1
            self._sockets.append(":".join("%02x" % b for b in mac_address))
            self._insert_mac_into_message(socket_id, sock_offset, mac_address)
            self._update_socket_status(socket_id, sock_offset, ServerOpCode.ACTIVE)
            response = ServerOpCode.OK

        return response

    def _remove_socket(self, socket_id: int, sock_offset: int, mac_address: bytes) -> ServerOpCode:
        response = ServerOpCode.OK
        if self._num_active_sockets:
            self._num_active_sockets -= 1
        try:
            self._sockets.remove(":".join("%02x" % b for b in mac_address))
        except ValueError as exception:
            pass

        self._socket_message_matrix[sock_offset][socket_id] = [0,0,0,0,0] 
        self._update_socket_status(socket_id, sock_offset, ServerOpCode.INACTIVE)        

        return response