from robot.strategy.behaviour import Selector, Sequence, BlackBoard, TaskStatus
from robot.strategy.actions.movement_behaviours import GoToPosition, StopAction, SpinTask
from robot.strategy.actions.state_behaviours import InState
from robot.strategy.strategy_utils import GameStates
from itertools import cycle
from robot.movement.definitions import OpCodes
from robot.strategy.actions.decorators import Timer, IgnoreSmoothing
import numpy as np

from robot.strategy.acceptance_radius import AcceptanceRadiusEnum


class CalibrationTree(Selector):
    def __init__(self, name="behave"):
        super().__init__(name)
        self.waypoints_list = cycle([(37, 25), (117, 25), (117, 105), (37, 105)])
        stop_sequence = Sequence('Stop Sequence')
        stop_sequence.children.append(InState('Stopped Game?', GameStates.STOPPED))
        stop_sequence.children.append(StopAction('Wait'))
        self.children.append(stop_sequence)

        patrol = Sequence('Patrol')
        self.straight_line_movement = GoToPosition(target_pos=next(self.waypoints_list), 
                                                   max_speed=45, 
                                                   acceptance_radius=AcceptanceRadiusEnum.SMALL.value)

        ignore_smoothing = IgnoreSmoothing(name="Ignore smoothing pid")
        ignore_smoothing.add_child(self.straight_line_movement)
        patrol.children.append(ignore_smoothing)
        spin_task = Timer(exec_time=3)
        spin_task.add_child(SpinTask())
        #patrol.children.append(spin_task)

        self.children.append(patrol)

    def run(self, blackboard):
        status, action = super().run(blackboard)
        if status == TaskStatus.SUCCESS:
            self.straight_line_movement.set_position(next(self.waypoints_list))
            action = (OpCodes.STOP, .0, 0, .0)

        return status, action

