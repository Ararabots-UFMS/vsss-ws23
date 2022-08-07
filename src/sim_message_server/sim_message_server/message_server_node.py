#!/usr/bin/python3
from sys import argv
from random import randint
from threading import Thread

import rclpy
from rclpy.node import Node

from sim_message_server.message_server import MessageServer

class MessageServerNode(Node):

    def __init__(self):
        super().__init__('message_server')

        try:
            owner_id = argv[1]
        except Exception as exception:
            owner_id = 'Player_' + str(randint(0, 99999))
            self.get_logger().fatal(str(exception))
        try:
            team_color = int(argv[3]) if argv[3] else 0
        except Exception as exception:
            team_color = 0
            self.get_logger().fatal(str(exception))

        server = MessageServer(node=self, owner_id=owner_id, simulator_mode=True, team_color=team_color)
    
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