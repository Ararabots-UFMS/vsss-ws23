#!/usr/bin/python3
import sys
import os
import fltk as fl

sys.path[0] = root_path = os.environ['ROS_ARARA_ROOT'] + "src/"
from interface.Controller.ConnectionController import ConnectionController
from utils.yaml_handler import YamlHandler

if __name__ == '__main__':
    """The sole purpose of this program is to set the ROSMASTER and ROS_IP when Roscore cant be initiated"""
    yaml_handler = YamlHandler()  # Create the yaml reader and writer
    game_opt = yaml_handler.read(root_path + "parameters/game.yml", escape=True)  # Load the game options
    window = ConnectionController(game_opt)  # Pass the game options to the controller
    window.show()
    fl.Fl.run()
    yaml_handler.write(game_opt, root_path + "parameters/game.yml")  # Saves the current network configuration
