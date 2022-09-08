#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from threading import Thread
from message_server.message_server import MessageServer
from sys import argv
from random import randint


class MessageServerNode(Node):

    def __init__(self):
        
        try:
            owner_id = argv[1]
        except Exception as exception:
            owner_id = 'Player_' + str(randint(0, 99999))

        super().__init__('message_server', namespace=owner_id)

        server = MessageServer(node=self, owner_id=owner_id)
    
        server_thread = Thread(target=server.loop, args=())
        server_thread.daemon = True
        server_thread.start()

def main(args=None):
    rclpy.init(args=args)
    
    message_server = MessageServerNode()

    try:
        rclpy.spin(message_server)
    except KeyboardInterrupt:
        print('server stopped cleanly')
    except BaseException:
        print('exception in server')
        raise
    finally:
        # Destroy the node explicitly
        # (optional - Done automatically when node is garbage collected)
        message_server.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()