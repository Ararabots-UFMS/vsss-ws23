#!/usr/bin/python3
import sys
import os
import fltk as fl

sys.path[0] = root_path = os.environ['ROS_ARARA_ROOT'] + "src/"
from interface.Controller.ConnectionController import ConnectionController
from utils.json_handler import JsonHandler

if __name__ == '__main__':
    """The sole purpose of this program is to set the ROSMASTER and ROS_IP when Roscore cant be initiated"""
    json_handler = JsonHandler()  # Create the json reader and writer
    game_opt = json_handler.read(root_path + "parameters/game.json", escape=True)  # Load the game options
    window = ConnectionController(game_opt)  # Pass the game options to the controller
    window.show()
    fl.Fl.run()
    json_handler.write(game_opt, root_path + "parameters/game.json")  # Saves the current network configuration
