from typing import Iterable

from robot.strategy.actions.decorators import InvertOutput, DoNTimes, StatusChanged, KeepRunning
from robot.strategy.actions.game_behaviours import *
from robot.strategy.actions.movement_behaviours import *
from robot.strategy.actions.state_behaviours import InState
from robot.strategy.arena_utils import ArenaSections, LEFT, inside_rectangle
from robot.strategy.base_trees import BaseTree
from robot.strategy.behaviour import *
from robot.strategy.strategy_utils import GameStates

from robot.strategy.acceptance_radius import AcceptanceRadiusEnum
from utils.math_utils import RAD2DEG


class AlignWithBall(TreeNode):
    def __init__(self, name: str = "AlignWithAxis",
                 max_speed: int = 0,
                 acceptance_angle: float = 0.0872665, align_with_ball: bool = False):

        super().__init__(name)
        self.max_speed = max_speed
        self._acceptance_angle = acceptance_angle

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        import math
        
        # angle_to_correct = angle_between(Vec2D.left(), blackboard.ball.position - blackboard.robot.position)
        to_ball_vec = blackboard.ball.position - blackboard.robot.position

        angle_to_correct = math.atan2(to_ball_vec[1], to_ball_vec[0])
        # rospy.logfatal(f"{angle_to_correct * RAD2DEG}")

        if abs(angle_to_correct - abs(blackboard.robot.orientation)) <= self._acceptance_angle:
            status = TaskStatus.SUCCESS
        else:
            status = TaskStatus.RUNNING
        return status, (OpCodes.SMOOTH + OpCodes.ORIENTATION_AVERAGE, angle_to_correct, self.max_speed, .0)


class IsInRangeOfDefense(TreeNode):
    def __init__(self, name: str = "Defense", look_ahead: float = 0):
        super().__init__(name)
        self.look_ahead = look_ahead

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:

        team_side = blackboard.home_goal.side
        ball_pos = blackboard.ball.position_prediction(self.look_ahead)

        # Trapézio incluindo seção do goleiro e uma área a frente do gol
        a, b = ( (0, 30), (45, 100) ) if team_side == 0 else ( (105, 30), (150, 100) )
        c, d = ( (0, 0), (15, 130) ) if team_side == 0 else ( (135, 0), (150, 130) )

        if inside_rectangle(a, b, ball_pos) or inside_rectangle(c, d, ball_pos):
            return TaskStatus.SUCCESS, NO_ACTION
        else:
            return TaskStatus.FAILURE, NO_ACTION


class GoalKeeper(BaseTree):
    def __init__(self, name: str = "behave"):
        super().__init__(name)

        self._lookahead = 2 # segundos

        normal = Sequence("Normal")
        self.add_child(normal)

        normal.add_child(InState("CheckNormalState", GameStates.NORMAL))
        normal_actions = Selector("NormalActions")
        normal.add_child(normal_actions)

        normal_actions.add_child(self._wait_for_danger_task())

        # Retira bola da região do goleiro
        normal_actions.add_child(self.get_ball_out_of_def_area())

        # normal_actions.add_child(self._ball_on_defense_side_tree())

    def _wait_for_danger_task(self) -> TreeNode:
        tree = Sequence("Wait for Danger Task")
        # tree.add_child(IsInAttackSide("VerifyBallInAttack", lambda b: b.ball.position))
        tree.add_child(InvertOutput(child=IsInRangeOfDefense()))
        tree.add_child(GoToGoalCenter(max_speed=45, acceptance_radius=AcceptanceRadiusEnum.LARGE.value)) # Univector é arriscado demais
        keep_align_action = KeepRunning()
        keep_align_action.add_child(AlignWithBall())
        tree.add_child(keep_align_action)
        return tree


    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        # self.set_y_axis(blackboard)
        #self._set_bottom_line_axis(blackboard)
        return super().run(blackboard)

    def get_ball_out_of_def_area(self) -> TreeNode:
        tree = Sequence("GetBallOutOfDefenseArea")

        tree.add_child(GoToBallUsingMove2Point(
            acceptance_radius=AcceptanceRadiusEnum.SMALL.value - 1,
            speed=75, speed_prediction=True
            )
        )

        push_or_spin = Selector("PushOrSpin")
        push_action = Sequence("PushAction")
        push_action.add_child(IsBehindBall("IsBehindBall", distance=13))
        push_action.add_child(ChargeWithBall(max_speed=100))

        push_or_spin.add_child(push_action)
        push_or_spin.add_child(SpinTask())

        tree.add_child(push_or_spin)

        return tree


class OutOfGoalAction(Sequence):
    def __init__(self, name: str = "Get Out Of Goal"):
        super().__init__(name)
        self.add_child(IsInsideDefenseGoal("IsGoalkeeperInsideGoal",
                                           lambda b : b.robot.position))
        self.add_child(GetOutOfGoal(max_speed=45, acceptance_radius=AcceptanceRadiusEnum.LARGE.value))

    def run(self, blackboard: BlackBoard):
        status, action = super().run(blackboard)
        if status == TaskStatus.SUCCESS:
            self.children[1].target_pos = None
        return status, action
