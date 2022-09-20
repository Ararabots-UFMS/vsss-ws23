import asyncio
import rclpy

class BluetoothDevice:

    def __init__(self, sock_id: int, mac_addr: str, node, shared_queue):
        self._sock_id  = sock_id
        self._mac_addr = mac_addr
        self.UART_RX_CHAR_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
        self._client = None
        self._node = node 
        self._queue = shared_queue

    async def main_loop(self):
        while asyncio.get_event_loop().is_running():
            msg_payload = await self._queue.get()
            self._node.get_logger().warning(f"BLE device {self._sock_id}({self._mac_addr}) with {repr(msg_payload)}!")
        self._node.get_logger().fatal(f"BLE device {self._sock_id}({self._mac_addr}) Disconnected")