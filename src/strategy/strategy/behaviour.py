from typing import List
from collections import deque
import time
from abc import abstractmethod, ABC
from enum import Enum
from typing import Tuple
import numpy as np
from itertools import count
from robot.movement.definitions import OpCodes
from strategy.arena_utils import RIGHT, LEFT
from strategy.strategy_utils import GameStates, BehavioralStates
from utils import physics
from utils.json_handler import JsonHandler
from utils.linalg import *

angle = distance = float
speed = int
ACTION = Tuple[OpCodes, angle, speed, distance]
NO_ACTION = (-1, 0, 0, 0)

class TaskStatus(Enum):
    SUCCESS = 0
    FAILURE = 1
    RUNNING = 2


class BlackBoard:
    """docstring for BlackBoard"""

    def __init__(self):
        self.game = Game()
        self.my_id = None
        self.current_orientation = None
        self.team_color = None
        self.advantage_team = 2 # None
        self.enemy_goal = Goal()
        self.home_goal = Goal()

        self.ball = physics.MovingBody()

        self.robot = FriendlyRobot()

        self.home_team = HomeTeam()
        self.enemy_team = EnemyTeam()

        self.current_automatic_position : int = 0
        self.automatic_positions = JsonHandler.read("parameters/automatic_positions.json", escape=True)

    def set_robot_variables(self, robot_id,
                                  robot_position, 
                                  robot_orientation,
                                  robot_speed,
                                  robot_vorientation):
        # TODO: Refatorar aqui para incluir novas informações do things_position
        self.robot.id = robot_id
        self.robot.position = Vec2D.from_array(robot_position)
        self.robot.orientation = robot_orientation
        self.robot._speed = Vec2D.from_array(robot_speed)
        self.robot.vorientation = robot_vorientation


    def __repr__(self):
        return 'BlackBoard:\n' + str(self.game) + "\n" + str(self.home_team) + str(self.enemy_team)


class TreeNode:
    def __init__(self, name, children=[]):
        self.name = name
        self.children = []
        for child in children:
            self.add_child(child)

    def add_child(self, node) -> None:
        self.children.append(node)

    @abstractmethod
    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        raise Exception("subclass must override run")


class Sequence(TreeNode):
    """
        A sequence runs each task in order until one fails,
        at which point it returns FAILURE. If all tasks succeed, a SUCCESS
        status is returned.  If a subtask is still RUNNING, then a RUNNING
        status is returned and processing continues until either SUCCESS
        or FAILURE is returned from the subtask.
    """

    def __init__(self, name, children: List[TreeNode] = []):
        super().__init__(name, children)

    def run(self, blackboard):
        for c in self.children:
            status, action = c.run(blackboard)
            if status != TaskStatus.SUCCESS:
                return status, action

        return TaskStatus.SUCCESS, NO_ACTION


class Selector(TreeNode):
    """
        A selector runs each task in order until one succeeds,
        at which point it returns SUCCESS. If all tasks fail, a FAILURE
        status is returned.  If a subtask is still RUNNING, then a RUNNING
        status is returned and processing continues until either SUCCESS
        or FAILURE is returned from the subtask.
    """

    def __init__(self, name: str, children: List[TreeNode] = []):
        super().__init__(name, children)

    def run(self, blackboard) -> Tuple[TaskStatus, ACTION]:

        for c in self.children:
            status, action = c.run(blackboard)
            if status != TaskStatus.FAILURE:
                return status, action

        return TaskStatus.FAILURE, NO_ACTION


class FriendlyRobot(physics.MovingBody):
    def __init__(self):
        super().__init__()
        self.id = -1 # Sem ID definido do simulador definido
        self.role = 0
        self.last_know_location = None

    def __setattr__(self, key, value):
        if key == 'position' and (value[0] or value[1]):
            self.last_know_location = value
        super().__setattr__(key, value)


class Goal:
    def __init__(self):
        self.side = LEFT
        self.position = Vec2D.origin()

    def __setattr__(self, key, value):
        if key == 'side':
            super().__setattr__(key, value)
            super().__setattr__('position', Vec2D(value * 150, 65))


class Game:
    def __init__(self):
        self.state = BehavioralStates.STOPPED
        self.meta_robot_id = 0
        self.freeball_robot_id = 0
        self.penalty_robot_id = 0

    def __repr__(self):
        return "-GameState: " + \
               "\n--state: " + str(self.state) + \
               "\n--meta_robot_id: " + str(self.meta_robot_id) + \
               "\n--freeball_robot_id: " + str(self.freeball_robot_id) + \
               "\n--penalty_robot_id: " + str(self.penalty_robot_id)


class Team(ABC):
    def __init__(self):
        self._ids = [-1 for _ in range(5)] # -1 para ID nao utilizado. Identificadores de robos no FIRASIM ("tags")
        self._positions = [Vec2D.origin() for _ in range(5)]
        self._speeds = [Vec2D.origin() for _ in range(5)]
        self._orientations = [0 for _ in range(5)]
        self._vorientations = [0 for _ in range(5)]
        self.robots = []
        self.number_of_robots = 0
        self.maximum_number_of_robots = 5

    def __repr__(self):
        return "--positions: " + str(self._positions) + \
               "\n--speeds: " + str(self._speeds) + \
               "\n--orientations: " + str(self._orientations) + \
               "\n--vorientations: " + str(self._vorientations) + \
               "\n--number_of_robots: " + str(self.number_of_robots) + \
               "\n--maximum_number_of_robots: " + str(self.maximum_number_of_robots) + \
               "\n--robots: " + str(self.robots)

    def create_new_robot(self):

        self._ids.append(-1)
        self._positions.append(Vec2D.origin())
        self._speeds.append(Vec2D.origin())
        self._orientations.append(0)
        self._vorientations.append(0)

    def set_team_variables(self, robot_ids,
                                 robot_positions, 
                                 robot_orientations,
                                 robot_speeds,
                                 robot_vorientations, 
                                 robot_tag_index=-1):

        # TODO: Refatorar aqui para incluir novas informações do things_position
        # TODO: Adicionar mais getters para os novos atributos

        self.number_of_robots = 0

        for tag_index, robot_id, robot_position, robot_orientation, robot_speed, robot_vorientation in \
             zip(count(), robot_ids, robot_positions, robot_orientations, robot_speeds, robot_vorientations):

            if np.any(robot_position) and tag_index != robot_tag_index:
                self._ids[self.number_of_robots] = robot_id
                self._positions[self.number_of_robots] = Vec2D.from_array(robot_position)
                self._orientations[self.number_of_robots] = robot_orientation

                self._speeds[self.number_of_robots] =  Vec2D.from_array(robot_speed)
                self._vorientations[self.number_of_robots] = robot_vorientation

                # TODO: Isto é redundante?
                self.robots[self.number_of_robots].id = robot_id
                self.robots[self.number_of_robots].position = Vec2D.from_array(robot_position)
                self.robots[self.number_of_robots].orientation = robot_orientation

                self.robots[self.number_of_robots]._speed = Vec2D.from_array(robot_speed)
                self.robots[self.number_of_robots].vorientation = robot_vorientation

                self.number_of_robots += 1


                if self.maximum_number_of_robots <= self.number_of_robots:
                    self.maximum_number_of_robots = self.number_of_robots
                    self.create_new_robot()

    def __len__(self):
        return self.number_of_robots

    def __getitem__(self, item):
        if self.number_of_robots:
            return self.robots[item]
        else:
            raise StopIteration

    @property
    def positions(self):
        return self._positions[:self.number_of_robots]

    @property
    def orientations(self):
        return self._orientations[:self.number_of_robots]

    @property
    def speeds(self):
        return self._speeds[:self.number_of_robots]


class EnemyTeam(Team):
    def __init__(self):
        super().__init__()
        self.robots = [physics.MovingBody() for _ in range(5)]

    def create_new_robot(self):
        self.robots.append(physics.MovingBody())
        super().create_new_robot()

    def __repr__(self):
        return "-EnemyTeam:\n" + super().__repr__()


class HomeTeam(Team):
    def __init__(self):
        super().__init__()
        self.robots = [FriendlyRobot() for _ in range(5)]

    def create_new_robot(self):
        self.robots.append(FriendlyRobot())
        super().create_new_robot()

    def __repr__(self):
        return "-HomeTeam:\n" + super().__repr__()
