from logging import debug
from typing import List, Union, NewType
import bluetooth
from bluetooth import BluetoothSocket, BluetoothError
from enum import Enum
from time import time, sleep
import rclpy
from rclpy.node import Node
from collections import namedtuple
import heapq as hp
from threading import Lock, Semaphore
#from ctypes import c_ubyte
from utils.socket_interfaces import SenderSocket

from sys_interfaces.srv import MessageServerService
from sim_message_server.opcodes import ServerOpCode
from sim_message_server.message_server_publisher import MessageServerPublisher

Message = namedtuple("Message", ["priority", "socket_id", "payload"])

Seconds = NewType('seconds', float)


import sim_cam.sim.packet_pb2 as packet_pb2
import sim_cam.sim.command_pb2 as command_pb2


class MessageServer:

    UDP_IP = ""
    UDP_PORT = 20011

    def __init__(self, node: Node,
                 owner_id: str = None, 
                 simulator_mode: bool = False,
                 team_color: int = 0,
                 max_sockets_capacity: int = 5,
                 max_queue_size: int = 5,
                 socket_timeout: Seconds = 0.00064
                 ):

        self._node = node

        self._simulator_mode = simulator_mode
        self._capacity = max_sockets_capacity
        self._priority_queue = []
        self._max_queue_size = max_queue_size
        self.socket_timeout = socket_timeout

        self._adapter_lock = Lock()
        self._buffer_lock = Lock()
        self._socket_dic_lock = Lock()
        self._server_semaphore = Semaphore(0)

        self.TAG = "MESSAGE SERVER"

        self._sockets = {}
        self._num_active_sockets = 0
        self.default_port = 0x1001
        self._sockets_status = [0] * self._capacity
        suffix = '' if owner_id is None else '_' + owner_id
        self.service = self._node.create_service(MessageServerService,
                                    'message_server_service' + suffix,                                     
                                     self._service_request_handler)

        self.topic_publisher = MessageServerPublisher(self._node, owner_id)

        from sim_message_server.message_server_subscriber import MessageServerSubscriber
        self.topic_subscriber = MessageServerSubscriber(self, owner_id)

        self._last_check = time()

        # ============================================

        self.sock = SenderSocket.create(self.UDP_IP, self.UDP_PORT)

        # Pode ser um singleton mesmo?
        self.message = packet_pb2.Packet()        
        self.cmd = self.message.cmd.robot_commands.add()
        self.cmd.yellowteam = team_color     
        

    def _service_request_handler(self,
                                 request, response) -> int:
        response_value = ServerOpCode.ERROR
        if request.opcode == ServerOpCode.ADD.value:
            response_value = self._add_socket(request.socket_id, bytes(request.robot_mac_addr))

        elif request.opcode == ServerOpCode.REMOVE.value:
            response_value = self._remove_socket(request.socket_id)

        elif request.opcode == ServerOpCode.CHANGE_COLORS.value:
            response_value = self._change_team_color(request.socket_id)

        self.topic_publisher.publish(self._sockets_status)

        response.response = response_value

        return response

    def _add_socket(self, socket_id: int, mac_address: bytes) -> ServerOpCode:       
        # if self._simulator_mode:
        self._sockets[socket_id] = ("fake_mac", None)
        self._num_active_sockets += 1
        self._update_socket_status(socket_id, ServerOpCode.ACTIVE)
        # response = ServerOpCode.OK
        return ServerOpCode.OK

    def _mac_been_used(self, mac_address: bytes) -> bool:
        for mac, _ in self._sockets.values():
            if mac == mac_address:
                return True
        return False

    def _update_socket_status(self, sock_id: int, status: Enum) -> None:

        if status == ServerOpCode.ACTIVE:
            self._sockets_status[sock_id] = 1
        else:
            self._sockets_status[sock_id] = 0

    def _remove_socket(self, socket_id: int) -> ServerOpCode:
        #if self._simulator_mode:
        self._num_active_sockets -= 1
        self._update_socket_status(socket_id, ServerOpCode.INACTIVE)
        return ServerOpCode.OK


    def _change_team_color(self, socket_id: int) -> ServerOpCode:
        response = ServerOpCode.OK
        self.cmd.yellowteam = socket_id        
        return response

    def _lock_sockets(self):
        # self._socket_dic_lock.acquire()
        pass

    def _release_sockets(self):
        # self._socket_dic_lock.release()
        pass

    def loop(self) -> None:
        while rclpy.ok():
            self._server_semaphore.acquire()
            message = self._getItemFromBuffer()

            if message is not None:
                self.send_message(message.socket_id, message.payload)

            if time() - self._last_check > 1.0:
                self.topic_publisher.publish(self._sockets_status)
                self._last_check = time()

        self.on_shutdown()

    def _getItemFromBuffer(self) -> Message:
        m = None
        self._buffer_lock.acquire()
        if len(self._priority_queue) > 0:
            m = hp.heappop(self._priority_queue)
        self._buffer_lock.release()
        return m

    def putItemInBuffer(self, message: Message) -> None:
        self._buffer_lock.acquire()
        if len(self._priority_queue) == self._max_queue_size:
            self._priority_queue = hp.nsmallest(self._max_queue_size - 1,
                                                self._priority_queue)
            hp.heapify(self._priority_queue)
        hp.heappush(self._priority_queue, message)
        self._buffer_lock.release()
        self._server_semaphore.release()

    def send_message(self, id_: int, payload: List) -> None:
        # released = False

        if self._simulator_mode or (id_ in self._sockets.keys()):
            self._sim_send(id_, payload)

    def on_shutdown(self) -> None:
        for _, socket in self._sockets.values():
            self._close(socket)

    def _connect(self, sock: BluetoothSocket, mac: str, port: int) -> None:
        self._adapter_lock.acquire()
        try:
            sock.connect((mac, port))
        except:
            self._adapter_lock.release()
            raise IOError
        self._adapter_lock.release()

    def _send(self, sock: BluetoothSocket, payload: List) -> None:
        self._adapter_lock.acquire()
        try:
            n = sock.send(bytes(payload))
        except Exception as e:
            self._adapter_lock.release()
            raise e

        self._adapter_lock.release()

    def _sim_send(self, id_: int, payload: List):
        self._adapter_lock.acquire()
        self.cmd.id = id_        
        # rospy.logfatal(f"{payload[0]} {payload[1]} {payload[2]}")

        # self.LEFTFORWARD_RIGHTFORWARD = 0x00  # 0000 0000
        # self.LEFTFORWARD_RIGHTBACKWARD = 0x01  # 0000 0001
        # self.LEFTBACKWARD_RIGHTFORWARD = 0x02  # 0000 0010
        # self.LEFTBACKWARD_RIGHTBACKWARD = 0x03  # 0000 0011
    
        # O payload passou a ser um vetor float32[3] para simulação.
        # Por isso, é necessário fazer um cast para int na primeira posição antes de fazer a operação de bits
        self.cmd.wheel_left =  -payload[1] if (int(payload[0])&2) else payload[1] 
        self.cmd.wheel_right = -payload[2] if (int(payload[0])&1) else payload[2]
        
        payload = self.message.SerializeToString()
        self.sock.sendall(payload)
        self._adapter_lock.release()
        # rospy.logfatal(f"{id_} {payload[1]}")

    def _close(self, sock: BluetoothSocket) -> None:
        self._node.get_logger().fatal("REMOVING SOCKET: " + repr(sock))        
        self._adapter_lock.acquire()
        sock.close()
        sleep(self.socket_timeout)
        self._adapter_lock.release()
