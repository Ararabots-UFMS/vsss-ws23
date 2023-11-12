#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from threading import Thread
from now_message_server.message_server import MessageServer
from sys import argv
from random import randint
import platform

class MessageServerNode(Node):

    def __init__(self):
        
        user_namespace = platform.node().replace('-','_')

        super().__init__('message_server', namespace=user_namespace)

        self.server = MessageServer(node=self, owner_id=user_namespace)


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
        # message_server.server.end_message_server()
        message_server.server.serial_writer.close()
        message_server.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()