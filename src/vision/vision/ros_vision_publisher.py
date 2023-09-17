#!/usr/bin/env python
import rclpy
from rclpy.node import Node
#!/usr/bin/env python
PKG = 'verysmall'
import numpy as np
from sys_interfaces.msg import ThingsPosition
from sys_interfaces.srv import VisionCommand
from rclpy.qos import QoSPresetProfiles

class RosVisionService:
    """
    This Class implements a service for changing vision parameters
    """
    def __init__(self, node: Node, _request_function):
        """
        :param _request_function: function callback
        """
        self._node = node
        self.core = self._node.create_service(
            VisionCommand, 
            '/vision_command', 
            _request_function)


class RosVisionPublisher:
    """
    This class can publish Vision messages on a Things Position Topic
    """
    def __init__(self, node: Node, isnode=False):       
        #
        self._node = node
        self.pub = self._node.create_publisher(
            ThingsPosition, 
            '/things_position', 
            qos_profile=QoSPresetProfiles.SENSOR_DATA.value)

    def publish(self, ball_pos, yellow_pos, yellow_orient, blue_pos, 
                blue_orientation, fps):

        """
            This function publishes in the things position topic

            :param ball_pos: int16[2]
            :param yellow_pos: int16[10]
            :param yellow_orient: int16[5]
            :param blue_pos: int16[10]
            :param blue_orientation: int16[5]
            :param fps: utin16
            :return: returns nothing
        """

        msg = ThingsPosition()
        
        msg.ball_pos =    np.float32(ball_pos * 100).tolist()
        msg.yellow_team_pos = np.float32(yellow_pos.flatten() * 100).tolist()
        msg.yellow_team_orientation = np.float32(yellow_orient.flatten() * 10000).tolist()
        msg.blue_team_pos = np.float32(blue_pos.flatten() * 100).tolist()
        msg.blue_team_orientation = np.float32(blue_orientation.flatten() * 10000).tolist()
        
        msg.vision_fps = int(fps * 100)
        
        try:
            self.pub.publish(msg)
        except Exception as excepetion:
            self._node.get_logger().fatal(excepetion)
            self._node.get_logger().fatal(msg)


if __name__ == '__main__':
    try:
        r = RosVisionPublisher(True)
        msg = ThingsPosition()
        msg.ball_pos = (100.0,100.0)
        msg.team_pos[0] = 50.0
        msg.team_pos[1] = 50.0
        while rclpy.ok():
            r.pub.publish(msg)
    except Exception as excepetion:
        pass
