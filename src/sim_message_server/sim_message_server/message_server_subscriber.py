import rospy

from verysmall.msg import message_server_topic
from message_server_module.message_server import MessageServer, Message


class MessageServerSubscriber:
    def __init__(self, my_server: MessageServer, owner_id: str = None):
        self._my_server = my_server
        suffix = '' if owner_id is None else '_'+owner_id
        rospy.Subscriber('message_server_topic' + suffix,
                         message_server_topic,
                         self._read_topic,
                         queue_size=5)

    def _read_topic(self, data: message_server_topic) -> None:        
        m = Message(data.priority, data.socket_id, data.payload)
        self._my_server.putItemInBuffer(m)
