#!/usr/bin/python3
# -*- coding: utf-8 -*-

from utils.json_handler import JsonHandler
from utils.model import Model
# from utils.process_killer import ProcessKiller
from interface.Controller.MainWindowController import MainWindowController
from interface.Controller.LoadingController import LoadingController
from utils.ros_utils import RosUtils
from interface.ros_game_topic_publisher import GameTopicPublisher
from interface.Coach.Coach import Coach
import rclpy
from argparse import ArgumentParser
from utils.camera_loader import CameraLoader
from random import randint
from sys import argv


"""
Instantiates all the windows, robots, topics and services
"""
def main(args=None):

    rclpy.init(args=args)

    try:
        owner_id = argv[1]
    except ValueError:
        owner_id = 'Player_' + str(randint(0, 99999))

    # Necessario no ROS2?
    # ProcessKiller(["robot"])

    node_obj = rclpy.create_node('virtual_field', namespace=owner_id)

    # Load the database
    model = Model()

    # Create roslaunch from API
    # launch = roslaunch.scriptapi.ROSLaunch()
    # launch.start()

    lc = LoadingController()
    lc.start()

    # vision_owner = True

    # if RosUtils.topic_exists("/things_position"):
    #     return_type, device_index = -1, -1
    #     vision_owner = False
    # else:
    #     # Create the GUI
    #     # Search for the usb camera, if not present, the program ask to a substitute
    #     # be a file or another camera
    #     lc.stop()
    #     return_type, device_index = CameraLoader(model.game_opt['camera']).get_index()

    #     lc.start("Carregando nó da visão")
    #     # Launch Vision with another Topic
    #     arguments = str(device_index) + " " + owner_id

    #     vision_node = roslaunch.core.Node('verysmall', 'vision_node.py',
    #                                       name='vision', args=arguments)

    #     # launches the node and stores it in the given memory space
    #     vision_process = launch.launch(vision_node)

    game_topic_publisher = GameTopicPublisher(node_obj, model.game_opt, model.robot_params, model.robot_roles,
                                              owner_id)

    coach = Coach(node_obj, model, game_topic_publisher)

    lc.stop()
    controller = MainWindowController(node_obj, model, coach, game_topic_publisher)

    lc = LoadingController()
    lc.start("Salvando banco de dados")

    # if vision_owner:
    #     vision_process.stop()

    model.save_params()
    lc.stop()

    node_obj.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()