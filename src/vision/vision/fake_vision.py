import rclpy
from rclpy.node import Node
import sys
import cv2

import os
from enum import Enum
from ros_vision_publisher import RosVisionPublisher
from ros_vision_publisher import RosVisionService
from sys_interfaces.msg import ThingsPosition

class VisionOperations(Enum):
    """
    This class stores vision operations for service request
    """
    SHOW = 1
    CROPPER = 2
    COLOR_CALIBRATION = 3
    SET_TEAM_COLOR_BLUE = 4
    SET_TEAM_COLOR_YELLOW = 5

class VisionNode(Node):
    """
    A node for spinning the Vision
    """
    def __init__(self):
        super().__init__('vision')
        self.mercury = RosVisionPublisher(self, True)
        self.msg = ThingsPosition()

        self.msg.yellow_team_pos[0] = 75
        self.msg.yellow_team_pos[1] = 75
        self.ball_pos = [0, 0]
        self.ball_pos[0] = 5
        self.ball_pos[1] = 2
        self.state = 0

        # Creates the service responsible for vision modes and operations
        self.service = RosVisionService(self.vision_management)

    def tick(self):

        if self.state == 0:
            self.ball_pos[0] += 1
            if self.ball_pos[0] == 145:
                self.state = 1
        elif self.state == 1:
            self.ball_pos[1]+= 1
            if self.ball_pos[1] == 130:
                self.state = 2
        elif self.state == 2:
            self.ball_pos[0]-=1
            if self.ball_pos[0] == 5:
                self.state = 3
        else:
            self.ball_pos[1]-=1
            if self.ball_pos[1] == 2:
                self.state = 0

        self.msg.ball_pos[0] = self.ball_pos[0]*100
        self.msg.ball_pos[1] = self.ball_pos[1]*100

        self.mercury.pub.publish(self.msg)


    def vision_management(self, req):
        """
        This is the reading function for a service response
        :param req: variable to get the request operation
        :return: bool
        """
        success = True
        self.state_changed = req.operation
        return success

def main(args=None):
    rclpy.init(args=args)

    vision_node = VisionNode()
    rate = vision_node.create_rate(30)  # 30hz

    while not rclpy.ok():
        vision_node.tick()
        rate.sleep()
    
    vision_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()