from strategy.actions.game_behaviours import *
from strategy.actions.movement_behaviours import MarkBallOnAxis, GoToBallUsingUnivector, SpinTask, \
    GoToBallUsingMove2Point, ChargeWithBall, GoBack, FollowAlly, GoToDefenseRange, GoToPosition, AlignWithAxis
from strategy.actions.state_behaviours import InState
from strategy.actions.decorators import InvertOutput
from strategy.base_trees import BaseTree
from strategy.behaviour import *
from strategy.strategy_utils import GameStates
from strategy.acceptance_radius import AcceptanceRadiusEnum


class Defender(BaseTree):

    def __init__(self, name="behave"):
        super().__init__(name)

        normal = Sequence("Normal")
        normal.add_child(InState('CheckNormalState', GameStates.NORMAL))

        defend = Selector("Defend")

        border = Sequence("Border")
        border.add_child(IsBallInRangeOfDefense("RangeOfDefense"))
        border.add_child(IsBallInDefenseBorder("BallInBorder"))
        border.add_child(GoToBallUsingMove2Point("Move2Point", speed=60, 
        acceptance_radius=AcceptanceRadiusEnum.DEFAULT.value))
        border.add_child(SpinTask("Spin"))
        defend.add_child(border)

        middle = Sequence("Middle")
        middle.add_child(IsBallInRangeOfDefense("InRangeOfDefense"))
        middle.add_child(GoToBallUsingUnivector("UsingUnivector", 
        acceptance_radius=AcceptanceRadiusEnum.DEFAULT.value, max_speed=60,
                                                speed_prediction=False))
        middle.add_child(ChargeWithBall("ChargeWithBall"))

        defend.add_child(middle)

        ball_near_goal_check = Sequence("CanRobotUseMove2PointToRecoverBall?")
        ball_near_goal_check.add_child(CanDefenderUseMove2PointToRecoverBall())
        ball_near_goal_check.add_child(GoToBallUsingMove2Point("Move2Point", speed=60, 
        acceptance_radius=AcceptanceRadiusEnum.DEFAULT.value))

        defend.add_child(self.ball_on_goalkeeper_section_tree())

        mark = Sequence("Mark")

        reposition = Selector("reposition")
        kick = Sequence("Kick")
        kick.add_child(IsNearBall("nearBall"))
        kick.add_child(SpinTask("Spin"))

        reposition.add_child(kick)
        reposition.add_child(AmIInDefenseField("AmIInAttackField"))
        reposition.add_child(GoBack("GoBack", 
        acceptance_radius=AcceptanceRadiusEnum.LARGE.value))

        mark.add_child(reposition)

        mark.add_child(self.go_to_defender_area_then_mark())

        defend.add_child(mark)
        normal.add_child(defend)
        self.add_child(normal)
    
    # Waits for the goalkeeper to get the ball out of the goalkeeper's section
    def ball_on_goalkeeper_section_tree(self) -> TreeNode:
        tree = Sequence("Ball on goakeeper section")
        tree.add_child(IsBallInGoalkeeperSection())

        #self._goalkeeper_section_task = GoToPositionUsingUnivector(max_speed=50,
        #                                acceptance_radius=AcceptanceRadiusEnum.NORMAL.value)
                                        
        self._goalkeeper_section_task = FollowAlly(ally_id=0, max_speed=30, acceptance_radius=25) #37
                                                                        
        tree.add_child(self._goalkeeper_section_task)
        tree.add_child(AlignWithAxis())
        return tree

    # Mark ball when it is in enemy side
    def go_to_defender_area_then_mark(self):
        tree = Sequence("GoToDefenderAreaThenMark")
        tree.add_child(GoToDefenseRange(blackboard_key='robot', speed=75))

        # TODO: Validar estas alterações -> zagueiro se move em direção ao y da bola em um X fixo (olhar self.run).
        # Depois de se mover, forçamos alinhamento com o eixo Y para facilitar movimentação
        self._mark_ball_task = GoToPosition(max_speed=60, acceptance_radius=AcceptanceRadiusEnum.LARGE.value)
        tree.add_child(self._mark_ball_task)
        tree.add_child(AlignWithAxis())

        # tree.add_child(MarkBallOnAxis("MarkBallOnAxis", predict_ball=False))
        return tree

    def _prepare_positions(self, blackboard: BlackBoard) -> None:

        team_side = blackboard.home_goal.side

        ball_on_attack_side_offset = (-1 + 2*team_side) * 25
        ball_on_goalkeeper_section_offset = (-1 + 2*team_side) * 40
        
        ball_y = blackboard.ball.position[1]
        self._mark_ball_task.set_position(Vec2D(75+ball_on_attack_side_offset, ball_y))
        # self._goalkeeper_section_task.set_position(Vec2D(75+ball_on_goalkeeper_section_offset, 65))

    
    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        self._prepare_positions(blackboard)

        # from utils.watcher import watcher
        # str_out = ""
        # for robot in blackboard.home_team.robots:
        #     str_out += f"id: {robot.id} //"
        # watcher.print(str_out)

        status, action = super().run(blackboard)
        return status, action

