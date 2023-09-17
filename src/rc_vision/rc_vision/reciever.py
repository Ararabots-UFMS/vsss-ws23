import socket
import struct
from time import time
import rc_vision.wrapper_pb2 as wr


import rclpy
from rclpy.node import Node

from sys_interfaces.msg import ThingsPosition # CHANGE

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class VSS_vision(Node):
    def __init__(self, vision_ip = "224.5.23.2", vision_port = 10015):
        super().__init__('vision')
        self.vision_ip = vision_ip
        self.vision_port = vision_port

        self.vision_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.vision_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        self.vision_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 128)
        self.vision_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
        self.vision_sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, struct.pack("=4sl", socket.inet_aton(self.vision_ip), socket.INADDR_ANY))
        self.vision_sock.bind((self.vision_ip, self.vision_port))

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
                data,_ = self.vision_sock.recvfrom(1024)
            except Exception as e:
                print(e)
            
            if data!=None:
                break

        if data!=None:
            decoded_data = wr.SSL_WrapperPacket().FromString(data)
        
        self.frame = decoded_data.detection

        pos_offset = 0
        for robot in self.frame.robots_yellow:
            self.msg.yellow_team_ids[int(pos_offset/2)] = robot.robot_id
            self.msg.yellow_team_pos[pos_offset]       = (robot.x*10 + 7500)
            self.msg.yellow_team_pos[pos_offset + 1]   = (robot.y*10 + 6500)
            self.msg.yellow_team_orientation[int(pos_offset/2)] = robot.orientation * 10000
            pos_offset += 2
            # print(self.frame.robots_yellow)

        pos_offset = 0
        for robot in self.frame.robots_blue:
            self.msg.blue_team_ids[int(pos_offset/2)] = robot.robot_id
            self.msg.blue_team_pos[pos_offset]        = (robot.x*10 + 7500)
            self.msg.blue_team_pos[pos_offset + 1]    = (robot.y*10 + 6500)
            self.msg.blue_team_orientation[int(pos_offset/2)] = robot.orientation * 10000
            pos_offset += 2 


        self.msg.ball_pos[0] = (self.frame.balls[0].x*10 + 7500)
        self.msg.ball_pos[1] = (self.frame.balls[0].y*10 + 6500)

        if self.context.ok():
            self.publisher_.publish(self.msg)
            # print(self.msg)

        
        # robots_blue = frame.robots_blue
        # robots_blue.add()
        # print(robots_blue[0])
        # robot_yellow = frame.robots_yellow
        # print(decoded_data)

def main(args=None):
    rclpy.init(args=args)

    vision = VSS_vision()
    while 1:
        vision.reciever_frame()