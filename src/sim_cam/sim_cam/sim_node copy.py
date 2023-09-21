# import imp
import rclpy
from enum import Enum
from rclpy.node import Node

from sys_interfaces.msg import ThingsPosition # CHANGE

import sim_cam.sim.packet_pb2 as packet_pb2
from utils.socket_interfaces import ReceiverSocket
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
class VisionOperations(Enum):
    """
    This class stores vision operations for service request
    """
    SHOW = 1
    CROPPER = 2
    COLOR_CALIBRATION = 3
    SET_TEAM_COLOR_BLUE = 4
    SET_TEAM_COLOR_YELLOW = 5


class MinimalPublisher(Node):

    UDP_IP = "224.5.23.2"
    UDP_PORT = 10015

    def __init__(self):
        super().__init__('vision')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.publisher_ = self.create_publisher(ThingsPosition, 'things_position', qos_profile=qos_profile)  # CHANGE
        self.msg = ThingsPosition()
        self.message = packet_pb2.Environment()
        
        self.sock = ReceiverSocket.create(self.UDP_IP, self.UDP_PORT)

    def tick(self):
       
        try:
            data, _ = self.sock.recvfrom(1024) # buffer size is 1024 bytes
        except KeyboardInterrupt:
            print('server stopped cleanly')
            return
        except BaseException:
            print('exception in server')
            raise

        self.message.ParseFromString(data)
        #esse laço do satanas ta mapeando os robos que estao separados na mensagem para um único array de posicoes que é o que temos 
        pos_offset = 0
        for robot in self.message.frame.robots_yellow:
            self.msg.yellow_team_ids[int(pos_offset/2)] = robot.robot_id
            self.msg.yellow_team_pos[pos_offset]       = (robot.x/0.8285*85 + 75)*100
            self.msg.yellow_team_pos[pos_offset + 1]   = (robot.y/0.6285*65 + 65)*100
            self.msg.yellow_team_speed[pos_offset]     = robot.vx
            self.msg.yellow_team_speed[pos_offset + 1] = robot.vy
            self.msg.yellow_team_orientation[int(pos_offset/2)] = robot.orientation * 10000
            self.msg.yellow_team_vorientation[int(pos_offset/2)] = robot.vorientation
            pos_offset += 2 

        pos_offset = 0
        for robot in self.message.frame.robots_blue:
            self.msg.blue_team_ids[int(pos_offset/2)] = robot.robot_id
            self.msg.blue_team_speed[pos_offset + 1]  = robot.vy
            self.msg.blue_team_pos[pos_offset]        = (robot.x/0.8285*85 + 75)*100
            self.msg.blue_team_pos[pos_offset + 1]    = (robot.y/0.6285*65 + 65)*100
            self.msg.blue_team_speed[pos_offset]      = robot.vx
            self.msg.blue_team_speed[pos_offset + 1]  = robot.vy
            self.msg.blue_team_orientation[int(pos_offset/2)] = robot.orientation * 10000
            self.msg.blue_team_vorientation[int(pos_offset/2)] = robot.vorientation
            pos_offset += 2 

        self.msg.ball_pos[0] = (self.message.frame.ball.x/0.8285*85 + 75)*100
        self.msg.ball_pos[1] = (self.message.frame.ball.y/0.6285 * 65 + 65)*100

        if self.context.ok():
            self.publisher_.publish(self.msg)
            print(self.msg)

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    while rclpy.ok():
        minimal_publisher.tick()

    minimal_publisher.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()