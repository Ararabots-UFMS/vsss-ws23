from sys_interfaces.msg import MessageServerTopic
from sim_message_server.message_server import MessageServer, Message


class MessageServerSubscriber:
    def __init__(self, my_server: MessageServer, owner_id: str = None):
        self._my_server = my_server
        suffix = '' if owner_id is None else '_'+owner_id
        my_server._node.create_subscription(
                         MessageServerTopic,
                         'message_server_topic' + suffix,
                         self._read_topic,
                         qos_profile=5)

    def _read_topic(self, data: MessageServerTopic) -> None:        
        m = Message(data.priority, data.socket_id, data.payload)
        self._my_server.putItemInBuffer(m)
