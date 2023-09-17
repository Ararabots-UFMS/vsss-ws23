import socket
import struct
from time import time
import vssref_command_pb2 as command
import vssref_common_pb2 as common


import rclpy
from rclpy.node import Node

from sys_interfaces.msg import ThingsPosition # CHANGE

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class VSS_referee(Node):
    def __init__(self, referee_ip = "224.5.23.2", referee_port = 10003):
        super().__init__('referee')
        self.referee_ip = referee_ip
        self.referee_port = referee_port

        self.referee_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.referee_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        self.referee_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 128)
        self.referee_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
        self.referee_sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, struct.pack("=4sl", socket.inet_aton(self.referee_ip), socket.INADDR_ANY))
        self.referee_sock.bind((self.referee_ip, self.referee_port))

        self.frame = None

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.publisher_ = self.create_publisher(ThingsPosition, 'things_position', qos_profile=qos_profile)  # CHANGE

        self.msg = ThingsPosition()
    
    def reciever_frame(self):
        data = None

        while True:
            try:
                data,_ = self.referee_sock.recvfrom(1024)
            except Exception as e:
                print(e)
            
            if data!=None:
                break

        if data!=None:
            decoded_data_command = command.VSSRef_Command().FromString(data)
            decoded_data_common_robot = common.Robot().FromString(data)
            decoded_data_common_frame = common.Frame().FromString(data)
        
        self.frame = decoded_data_command
        
        print("command:\n",self.frame)

        self.frame = decoded_data_common_robot

        print("com_robot:\n",self.frame)

        self.frame = decoded_data_common_frame

        print("com_frame:\n",self.frame)

        # pos_offset = 0
        # for robot in self.frame.robots_yellow:
        #     self.msg.yellow_team_ids[int(pos_offset/2)] = robot.robot_id
        #     self.msg.yellow_team_pos[pos_offset]       = (robot.x*10 + 7500)
        #     self.msg.yellow_team_pos[pos_offset + 1]   = (robot.y*10 + 6500)
        #     self.msg.yellow_team_orientation[int(pos_offset/2)] = robot.orientation * 10000
        #     pos_offset += 2
        #     # print(self.frame.robots_yellow)

        # pos_offset = 0
        # for robot in self.frame.robots_blue:
        #     self.msg.blue_team_ids[int(pos_offset/2)] = robot.robot_id
        #     self.msg.blue_team_pos[pos_offset]        = (robot.x*10 + 7500)
        #     self.msg.blue_team_pos[pos_offset + 1]    = (robot.y*10 + 6500)
        #     self.msg.blue_team_orientation[int(pos_offset/2)] = robot.orientation * 10000
        #     pos_offset += 2 


        # self.msg.ball_pos[0] = (self.frame.balls[0].x*10 + 7500)
        # self.msg.ball_pos[1] = (self.frame.balls[0].y*10 + 6500)

        # if self.context.ok():
        #     self.publisher_.publish(self.msg)

def main(args=None):
    rclpy.init(args=args)

    referee = VSS_referee()
    while 1:
        referee.reciever_frame()

if __name__ == '__main__':
    main(None)