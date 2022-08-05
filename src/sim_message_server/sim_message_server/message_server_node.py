#!/usr/bin/python3
from telnetlib import RCTE
import rclpy
from rclpy.node import Node

from threading import Thread
from sim_message_server.message_server import MessageServer

from sys import argv
from random import randint

class MessageServerNode(Node):

    def __init__(self):
        super().__init__('message_server')

        try:
            owner_id = argv[1]
        except ValueError:
            owner_id = 'Player_' + str(randint(0, 99999))
        # rospy.logfatal(f"AAAAAAAAAAAAAAa {argv[2]}")
        try:
            team_color = int(argv[3]) if argv[3] else 0
        except:
            team_color = 0

        server = MessageServer(node=self, owner_id=owner_id, simulator_mode = argv[2] == '1', team_color=team_color)
    
        server_thread = Thread(target=server.loop, args=())
        server_thread.daemon = True
        server_thread.start()


def main(args=None):
    rclpy.init(args=args)

    message_server = MessageServerNode()

    rclpy.spin(message_server)

    message_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()