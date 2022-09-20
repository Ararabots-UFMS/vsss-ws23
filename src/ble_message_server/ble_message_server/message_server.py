from typing import List, Union, NewType
import bluetooth
from bluetooth import BluetoothSocket, BluetoothError
from enum import Enum
from time import time, sleep
import rclpy
import asyncio
from rclpy.node import Node
from collections import namedtuple
import heapq as hp
from threading import Thread, Lock, Semaphore
from ctypes import c_ubyte

from ble_message_server.message_server_publisher import MessageServerPublisher
from ble_message_server.opcodes import ServerOpCode
from ble_message_server.ble_device import BluetoothDevice
from sys_interfaces.srv import MessageServerService

import numpy as np

Message = namedtuple("Message", ["priority", "counter", "socket_id", "payload"])

Seconds = NewType('seconds', float)


class MessageServer:

    def __init__(self, node: Node,
                 owner_id: str = None, 
                 max_sockets_capacity: int = 5,
                 max_queue_size: int = 5,
                 socket_timeout: Seconds = 0.00064):

        self._node = node

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
        self._sockets_status = np.zeros(self._capacity, dtype= np.uint8) # [0] * self._capacity
        suffix = '' if owner_id is None else '_' + owner_id
        self.service = self._node.create_service(MessageServerService,
                                    'message_server_service',                                     
                                     self._service_request_handler)

        self.topic_publisher = MessageServerPublisher(self._node, owner_id)

        from ble_message_server.message_server_subscriber import MessageServerSubscriber
        self.topic_subscriber = MessageServerSubscriber(self, owner_id)

        # self._last_check = time()
        self._node.create_timer(1, self._publish_status)
        self._loop_object = None
        self._devices = [None for _ in range(5)]
        self._tasks = [None for _ in range(5)]
        self._devices_queue = [asyncio.Queue(10) for _ in range(5)]

    def _publish_status(self):
        self.topic_publisher.publish(self._sockets_status)
        # self._node.get_logger().warning("Publish status Running!")

    def _service_request_handler(self,
                                 request, response) -> int:
        response_value = ServerOpCode.ERROR
        if request.opcode == ServerOpCode.ADD.value:
            response_value = self._add_socket(request.socket_id, bytes(request.robot_mac_addr))

        elif request.opcode == ServerOpCode.REMOVE.value:
            response_value = self._remove_socket(request.socket_id)

        # self.topic_publisher.publish(self._sockets_status)

        response.response = response_value.value

        return response

    def _add_socket(self, socket_id: int, mac_address: bytes) -> ServerOpCode:
        response = ServerOpCode.ERROR
        # self._loop = asyncio.get_event_loop()
        if self._num_active_sockets < self._capacity and \
                not self._mac_been_used(mac_address):

            mac_str = ":".join("%02x" % b for b in mac_address)

        device = BluetoothDevice(socket_id, mac_str, self._node, self._devices_queue[socket_id])
        self._devices[socket_id] = device
        self._tasks[socket_id] = self._loop_object.create_task(device.main_loop())
        self._update_socket_status(socket_id, ServerOpCode.ACTIVE)
        response = ServerOpCode.OK


        #     sock = BluetoothSocket(bluetooth.RFCOMM)

        #     try:
        #         self._connect(sock, mac_str, 1)
        #         #sock.settimeout(self.socket_timeout)

        #         self._sockets[socket_id] = (mac_str, sock)

        #         self._num_active_sockets += 1
        #         self._update_socket_status(socket_id, ServerOpCode.ACTIVE)
        #         response = ServerOpCode.OK
        #     except IOError as exception:
        #         sock.close()
        #         self._node.get_logger().fatal(self.TAG + ": ERROR IN ADDING SOCKET. " + repr(exception))

        return response

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
        response = ServerOpCode.OK
        self._tasks[socket_id].cancel()
        self._devices[socket_id] = None
        self._update_socket_status(socket_id, ServerOpCode.INACTIVE)
        # s = self._sockets.pop(socket_id, None)

        # if s is None:
        #     response = ServerOpCode.ERROR
        # else:
        #     self._num_active_sockets -= 1
        #     self._update_socket_status(socket_id, ServerOpCode.INACTIVE)
        #     socket = s[1]
        #     self._close(socket)

        return response

    def _lock_sockets(self):
        # self._socket_dic_lock.acquire()
        pass

    def _release_sockets(self):
        # self._socket_dic_lock.release()
        pass

    def end_message_server(self):
        self._future.set_result(True)

    def loop(self) -> None:
        asyncio.run(self._loop())

        self.on_shutdown()

    async def _loop(self):
        # self._server_semaphore.acquire()
        self._loop_object = asyncio.get_event_loop()
        # self._future =  self._loop_object.create_future()
        while rclpy.ok():
            await asyncio.sleep(1/120)
            # self._node.get_logger().fatal("Main loop Running!")
        # await self._future
        # message = self._getItemFromBuffer()

        # if message is not None:
        #     self.send_message(message.socket_id, message.payload)

        # if time() - self._last_check > 1.0:
        #     self.topic_publisher.publish(self._sockets_status)
        #     self._last_check = time()

    def _getItemFromBuffer(self) -> Message:
        m = None
        self._buffer_lock.acquire()
        if len(self._priority_queue) > 0:
            m = hp.heappop(self._priority_queue)
        self._buffer_lock.release()
        return m

    def putItemInBuffer(self, message: Message) -> None:
        # self._buffer_lock.acquire()
        # if len(self._priority_queue) == self._max_queue_size:
        #     self._priority_queue = hp.nsmallest(self._max_queue_size - 1,
        #                                         self._priority_queue)
        #     hp.heapify(self._priority_queue)
        # hp.heappush(self._priority_queue, message)
        # self._buffer_lock.release()
        # self._server_semaphore.release()
        try:
            self._devices_queue[message.socket_id].put_nowait(message.payload)
        except asyncio.QueueFull as exception:
            # self._node.get_logger().fatal(repr(exception))
            pass



    def send_message(self, id_: int, payload: List) -> None:
        released = False

        if id_ in self._sockets.keys():
            try:
                self._send(self._sockets[id_][1], payload)
            except Exception as e:
                self._node.get_logger().fatal("TIMEOUT EXCEPTION " + repr(e))
                self._remove_socket(id_)

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

    def _close(self, sock: BluetoothSocket) -> None:
        self._node.get_logger().fatal("REMOVING SOCKET: " + repr(sock))
        self._adapter_lock.acquire()
        sock.close()
        sleep(self.socket_timeout)
        self._adapter_lock.release()
