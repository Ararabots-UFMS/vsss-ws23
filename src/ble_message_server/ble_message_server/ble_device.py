import asyncio
import rclpy
from bleak import BleakClient
import numpy as np
import bleak
import time 
from collections import deque

class BluetoothDevice:

    def __init__(self, sock_id: int, mac_addr: str, node, shared_queue: deque):
        self._sock_id  = sock_id
        self._mac_addr = mac_addr
        self.MOTOR_POWER_UUID = "00002a07-0000-1000-8000-00805f9b34fb"
        self._client = None
        self._node = node 
        self._queue = shared_queue
        address = "B4:E6:2D:B5:B9:6B"
        # MODEL_NBR_UUID = "00002a24-0000-1000-8000-00805f9b34fb"        
        self.client = BleakClient(address, timeout=2.0)

    async def main_loop(self):
        
        try:
            await self.client.connect(timeout=2.0)
            #model_number = await self.client.read_gatt_char(self.MOTOR_POWER_UUID)
            #print(f"Model Number: {model_number}")

            while asyncio.get_event_loop().is_running():
                try:
                    start = time.process_time_ns()
                    msg_payload = self._queue.pop()
                    # self._queue.task_done()
                    await self.client.write_gatt_char(self.MOTOR_POWER_UUID, msg_payload, response=False) 
                    end = time.process_time_ns() 
                    self._node.get_logger().warning(f"BLE device {0}({(end-start)}ns)")           
                except IndexError as exception:
                    pass                
                # self._node.get_logger().warning(f"BLE device {self._sock_id}({self._mac_addr}) with {repr(msg_payload)}!")

        except Exception as e:
            print(e)
        finally:
            await self.client.disconnect()
            self._node.get_logger().fatal(f"BLE device {self._sock_id}({self._mac_addr}) Disconnected")
                