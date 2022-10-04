from sys_interfaces.msg import MessageServerTopic, GameTopic
from ble_message_server.message_server import MessageServer, Message


class MessageServerSubscriber:
    def __init__(self, my_server: MessageServer, owner_id: str = None):
        self._my_server = my_server
        suffix = '' if owner_id is None else '_'+owner_id
        self._internal_counter = 0
        my_server._node.create_subscription(
                         MessageServerTopic,
                         'message_server_topic',
                         self._read_topic,
                         qos_profile=5)

    def _read_topic(self, data: MessageServerTopic) -> None:
        self._internal_counter = (self._internal_counter + 1) & 7 # contador de 0 a 7(111)
        m = Message(data.priority, self._internal_counter, data.socket_id, data.payload)
        self._my_server.putItemInBuffer(m)
