from typing import List
import rospy
import numpy as np

from verysmall.msg import connection_status_topic


class MessageServerPublisher:
    def __init__(self, owner_id: str = None):
        self.TAG = "MESSAGE SERVER PUBLISHER"
        suffix = '' if owner_id is None else '_' + owner_id
        self.publisher = rospy.Publisher('connection_status_topic' + suffix,
                                         connection_status_topic,
                                         queue_size=10)

    def publish(self, sockets_status: List) -> None:
        msg = sockets_status
        try:
            self.publisher.publish(msg)
        except rospy.ROSException as e:
            rospy.logfatal(self.TAG + ": UNABLE TO PUBLISH. " + repr(e))
