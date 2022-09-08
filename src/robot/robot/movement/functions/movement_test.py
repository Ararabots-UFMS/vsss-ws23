import numpy as np
from movement import Movement
robot_position = np.array([0, 0])
robot_vector = np.array([1, 1])
goal_position = np.array([20, 0])
speed = 5
movement = Movement(10)
print(movement.move_to_point(robot_position, robot_vector, goal_position, speed))