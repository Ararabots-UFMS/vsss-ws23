import sys
import os
import numpy as np
sys.path[0] = root_path = os.environ['ROS_ARARA_ROOT'] + "src/robot/movement/"
from simulator.simulator import Simulator
from functions.movement import Movement
from univector.un_field import univectorField
import cv2
import numpy as np
import random

# define when use the mouse
mouseMode = False

# univector
RADIUS = 3.48  # Distance from ball
KR = 1
K0 = 0.12
DMIN = 15
LDELTA = 50
RIGHT = 1
LEFT = 0

#define the current game state
game_state = "Stop"

robotInitPosition = (200, 200)
ballInitPosition = (500, 500)
advRobotPosition = (350, 350)
lastRobotPosition = (200, 200)
robotSpeed = np.array([0, 0])

# window size
img = np.zeros((600, 800, 3), np.uint8)
# creating simulator
sim = Simulator(img)
# initialize arena
sim.initArena()
# initialize robot
sim.drawRobot(robotInitPosition, [1, 0])
# initialize ball
sim.drawBall(ballInitPosition)
# initialize adversary robot
sim.drawAdv(np.array(advRobotPosition))
# movement class
movement = Movement([30, 0, 0], 10)
movement.initialize_simulation()
# obstacles
obstacle = np.array(advRobotPosition)
vObstacle = np.array([0, 0])

# creates the univector field
univetField = univectorField(attack_goal=RIGHT)
univetField.update_constants(RADIUS, KR, K0, DMIN, LDELTA)
univetField.update_ball(np.array(sim.ball))
univetField.update_obstacles(obstacle, vObstacle)

leftSpeed = 0
rightSpeed = 0
done = False

def printInformation():
    print("**********************************************")
    print("         Initializing VSSS simulation         ")
    print("**********************************************")
    print("Press any key to start:")
    # print("To change the game state to normal game: n")
    # print("To change the game state to stop: s")
    # print("To change the game state to meta: m")
    # print("To change the game state to penalty: p")
    # print("To change the game state to free ball: f")
    print("To draw the ball in mouse position use: g")
    print("To exit the simulation press: q")
