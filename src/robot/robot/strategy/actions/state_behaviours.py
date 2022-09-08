from robot.strategy.behaviour import TaskStatus, Sequence, BlackBoard
from robot.strategy.strategy_utils import GameStates
from robot.movement.definitions import OpCodes


class InState:
    def __init__(self, name, _desired_state):
        self.name = name
        self.desired_state = _desired_state

    def run(self, blackboard):
        if self.desired_state == blackboard.game.state:
            return TaskStatus.SUCCESS, (OpCodes.STOP, 0, 0, 0)

        return TaskStatus.FAILURE, (OpCodes.STOP, 0, 0, 0)


class ChangeState:
    def __init__(self, name, _target_state):
        self.name = name
        self.target_state = _target_state

    def run(self, blackboard):
        blackboard.game.state = self.target_state
        return TaskStatus.FAILURE, (OpCodes.STOP, 0, 0, 0)
