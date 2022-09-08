from simulator_definition import *
import sys
import os
import numpy as np
sys.path[0] = root_path = os.environ['ROS_ARARA_ROOT'] + "src/strategy"
from attacker_with_univector.attacker_with_univector_controller import AttackerWithUnivectorController


def updateBallPosition(event, x, y, flags, param):
    if mouseMode:
        sim.drawBall((x, y))


if __name__ == "__main__":

    printInformation()
    # show img
    cv2.imshow('Robot Simulation', img)
    cv2.moveWindow('Robot Simulation', 400, 0)

    # define mouseCallBack
    cv2.setMouseCallback('Robot Simulation', updateBallPosition)

    key = cv2.waitKey(0)
    while 1:
        # avoid to clear the adversary
        sim.drawAdv(np.array(advRobotPosition))

        cv2.imshow('Robot Simulation', img)
        cv2.moveWindow('Robot Simulation', 400, 0)

        # use mouse
        if key == ord('g'):
            mouseMode = not mouseMode
        # end simulation
        elif key == ord('q'):
            cv2.destroyAllWindows()
            print("***************Simulation finished***************")
            break

        elif key == ord('s'):
            game_state = "Stop"
            #call state_machine

        elif key == ord('p'):
            game_state = "Penalty"
            #call state_machine

        elif key == ord('f'):
            game_state = "Free Ball"

        elif key == ord('n'):
            game_state = "Normal game"

        elif key == ord('m'):
            game_state = "Meta"

        lastRobotPosition = sim.robot

        leftSpeed, rightSpeed, done = movement.move_to_point(100, np.array(sim.robot), np.array(sim.robotVector), np.array(sim.ball))
        
        if not done:
            # move function
            sim.move(leftSpeed, rightSpeed)
            robotSpeed = np.array(sim.robot) - np.array(lastRobotPosition)

        # 60fps
        key = cv2.waitKey(16)
