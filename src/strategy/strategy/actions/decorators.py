import time
from abc import abstractmethod
from typing import Tuple
import numpy as np
import math as mth

from strategy.behaviour import TaskStatus, BlackBoard, ACTION, TreeNode
from robot.movement.definitions import OpCodes
from utils.math_utils import DEG2RAD, BACKWARDS, FORWARD
from robot.movement.definitions import OpCodes


class Decorator:
    def __init__(self, name: str, child=None):
        self.name = name
        self.child = child

    def add_child(self, child):
        self.child = child

    @abstractmethod
    def run(self, blackboard: BlackBoard):
        pass


class IgnoreFailure(Decorator):
    def __init__(self, name: str = 'IgnoreFailure'):
        super().__init__(name)

    def run(self, blackboard: BlackBoard):
        status, action = self.child.run(blackboard)
        if status == TaskStatus.RUNNING:
            return TaskStatus.RUNNING, action

        return TaskStatus.SUCCESS, action


class InvertOutput(Decorator):
    def __init__(self, name: str = 'Not', child = None):
        super().__init__(name, child)

    def run(self, blackboard: BlackBoard):
        if self.child is None:
            return TaskStatus.FAILURE, (OpCodes.INVALID, 0, 0, 0)
        else:
            status, action = self.child.run(blackboard)

            if status != TaskStatus.RUNNING:
                if status == TaskStatus.FAILURE:
                    return TaskStatus.SUCCESS, action
                else:
                    return TaskStatus.FAILURE, action

            return status, action


class Timer(Decorator):
    def __init__(self, name: str = 'Timeout', exec_time: float = 0):
        super().__init__(name)
        self.exec_time = exec_time
        self.initial_time = time.time()
        self.current_time = None

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        if self.child is None:
            return TaskStatus.FAILURE, (OpCodes.INVALID, 0, 0, 0)
        else:
            self.current_time = time.time()
            if self.current_time - self.initial_time < self.exec_time:
                return self.child.run(blackboard)
            else:
                self.initial_time = time.time()
                return TaskStatus.SUCCESS, (OpCodes.INVALID, 0, 0, 0)


class IgnoreSmoothing(Decorator):
    def __init__(self, name: str = "IgnoreSmoothing"):
        super().__init__(name)

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        if self.child is None:
            return TaskStatus.FAILURE, (OpCodes.INVALID, 0, 0, 0)
        else:
            status, action = self.child.run(blackboard)
            if action[0] == OpCodes.SMOOTH:
                action = (OpCodes.NORMAL, action[1], action[2], action[3])
            return status, action


class DoNTimes(Decorator):
    def __init__(self, name: str = "Do N times", n: int = 1):
        super().__init__(name)
        self.n = n

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        if self.child is None:
            return TaskStatus.FAILURE, (OpCodes.INVALID, 0, 0, 0)
        else:
            if self.n > 0:
                status, action = self.child.run(blackboard)
                if status != TaskStatus.RUNNING:
                    self.n -= 1
                return status, action
            return TaskStatus.FAILURE, (OpCodes.INVALID, 0, 0, 0)


class KeepRunning(Decorator):
    def __init__(self, name: str = "KeepRunningDecorator"):
        super().__init__(name)

    def run(self, blackboard: BlackBoard):
        _, action = self.child.run(blackboard)
        return TaskStatus.RUNNING, action


class StatusChanged(Decorator):
    def __init__(self, name: str = "StatusChanged", function=None):
        super().__init__(name)
        self.last_status = TaskStatus.SUCCESS
        self._function = function

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        status, action = self.child.run(blackboard)

        if status != self.last_status:
            self._function()

        self.last_status = status

        return status, action


class SafeHeadOnBorder(Decorator):
    def __init__(self, name: str = "SafeHeadOnBoarder", child = None):
        super().__init__(name, child)
    
    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        status, action = self.child.run(blackboard)

        y = blackboard.robot.position[1]
        if y < 4 or y > 126:
            if 70*DEG2RAD < abs(blackboard.robot.orientation) < 110*DEG2RAD:
                action = list(action)

                p = blackboard.robot.position
                o = blackboard.robot.orientation
                safe_head = self.get_safe_head(p, o)

                if safe_head == FORWARD:
                    action[0] = action[0] + OpCodes.USE_FORWARD_HEAD
                else:
                     action[0] = action[0] + OpCodes.USE_BACKWARD_HEAD

                action = tuple(action)
        
        return status, action
    
    def get_safe_head(self, position: np.ndarray, orientation):
        if position[1] < 65: #down border
            if abs(orientation - mth.pi) < abs(-orientation - mth.pi):
                return FORWARD
            else:
                return BACKWARDS
        else:
            if abs(orientation + mth.pi) < abs(-orientation + mth.pi):
                return FORWARD
            else:
                return BACKWARDS

class UseFrontHead(Decorator):
    def __init__(self, name: str = "UseFrontHead", child: TreeNode = None):
        super().__init__(name, child)
    
    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        status, action = self.child.run(blackboard)
        opcode = action[0] + OpCodes.USE_FORWARD_HEAD
        
        return status, (opcode,) + action[1:] 