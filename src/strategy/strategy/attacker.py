import math as mth
# import rospy

from strategy.actions.game_behaviours import IsBallInGoalkeeperSection, IsBehindBall,\
 IsRobotInsideEnemyGoalLine, IsBallInsideSections, IsNearBall,\
 IsBallInBorder, IsRobotInsideSections, \
 AmIAttacking 

from strategy.acceptance_radius import AcceptanceRadiusEnum

from strategy.actions.movement_behaviours import GoToBallUsingUnivector,\
     SpinTask, ChargeWithBall, GoToBallUsingMove2Point, CanUseMoveToPointSafely,\
     GoToPositionUsingUnivector, GoToPosition, AlignWithAxis, FollowAlly, StopAction, \
     RecoverBallUsingUnivector

from strategy.actions.state_behaviours import InState
from strategy.base_trees import BaseTree, FreeWayAttack
from strategy.behaviour import *
from strategy.strategy_utils import GameStates
from strategy.arena_utils import ArenaSections, univector_pos_section
from strategy.actions.decorators import SafeHeadOnBorder, InvertOutput


from utils.math_utils import FORWARD, BACKWARDS, DEG2RAD
from robot.movement.definitions import OpCodes


class Attacker(BaseTree):

    def __init__(self, name='behave'):
        super().__init__(name)
        self._critical_position_task = None
        self._move_to_point_task = None


        # Checa estado do jogo e estabelece o comportamento padrão/normal
        normal = SafeHeadOnBorder(child=Sequence('Normal'))
        normal.add_child(InState('CheckNormalState', GameStates.NORMAL))

        # normal.add_child(self.ball_on_goalkeeper_section_tree())


        main = Selector(name="Main behaviours")
        # Bola na seção do goleiro
        main.add_child(self.ball_on_goalkeeper_section_tree())

        # Verificar se o robo é o atacante principal
        main_attacker = Sequence("Main attacker behaviours")
        main_attacker.add_child(AmIClosestToBall())

        # Ações do atacante principal
        main_attacker_actions = Selector('Attacker Actions')
        # main_attacker_actions.add_child(FreeWayAttack('FreewayAttack'))
        main_attacker_actions.add_child(self._robot_inside_enemy_goal())
        main_attacker_actions.add_child(self.ball_and_robot_in_enemy_goalline())
        main_attacker_actions.add_child(self._ball_on_border_tree())
        main_attacker_actions.add_child(self.naive_go_to_ball())

        main_attacker.add_child(main_attacker_actions)
        main.add_child(main_attacker)
        # main.add_child(StopAction())
        main.add_child(
            FollowMainAttacker(
                max_speed=80,
                acceptance_radius=40
            )
        )

        normal.add_child(main)
        
        self.add_child(normal)

    def naive_go_to_ball(self) -> TreeNode:
        # tree = Selector("Go ball")

        middle = Sequence("Ball out of border")
        # middle.add_child(FreeWayAttack("Freeway"))
        univector_movement = GoToBallUsingUnivector("AttackBallInTheMiddle",
                                            max_speed=100,
                                            acceptance_radius=AcceptanceRadiusEnum.LARGE.value+1,
                                            speed_prediction=True)

        middle.add_child(univector_movement)
        middle.add_child(FreeWayAttack("AAAAAAAAAAAAa"))

        # spin_or_dash = Selector("SpinOrDash")

        # dash_sequence = Sequence("DashSequence")
        # dash_sequence.add_child(IsBehindBall("Naive dash", 60))
        # dash_sequence.add_child(ChargeWithBall(max_speed=255))

        # spin_or_dash.add_child(dash_sequence) 
        # spin_or_dash.add_child(SpinTask())
        # # middle.add_child(spin_or_dash)

        # tree.add_child(middle)

        return middle

    def ball_and_robot_in_enemy_goalline(self) -> TreeNode:
        tree = Sequence("BallAndRobotInAttackGoalLine")
        tree.add_child(IsRobotInsideEnemyGoalLine("EnemyGoalLine"))
        spin_or_dash = Selector("SpinOrDash")
        tree.add_child(spin_or_dash)

        spin_sequence = Sequence("SpinSequence")
        spin_or_dash.add_child(spin_sequence)

        dash_sequence = Sequence("DashSequence")
        spin_or_dash.add_child(dash_sequence)

        spin_sequence.add_child(
            IsBallInsideSections(sections=[ArenaSections.LEFT_DOWN_CORNER, 
                                           ArenaSections.LEFT_UP_CORNER,
                                           ArenaSections.RIGHT_DOWN_CORNER,
                                           ArenaSections.RIGHT_UP_CORNER]))
        spin_sequence.add_child(IsNearBall(acceptance_radius=AcceptanceRadiusEnum.NORMAL.value))
        spin_sequence.add_child(SpinTask())

        dash_sequence.add_child(IsBehindBall("IsBehindBall", 65))
        dash_sequence.add_child(ChargeWithBall("Attack", 200))

        return tree
    
    def _ball_on_border_tree(self) -> TreeNode:
        tree = Sequence("Ball on border")
        tree.add_child(IsBallInBorder())
        tree.add_child(CanUseMoveToPointSafely())  
        tree.add_child(GoToBallUsingMove2Point("GotoBallMove2point", speed=100,acceptance_radius=AcceptanceRadiusEnum.LARGE.value))
        tree.add_child(SpinTask('Spin'))

        return tree


    def _robot_inside_enemy_goal(self) -> TreeNode:
        tree = Sequence("RobotInsideEnemyGoal")
        tree.add_child(IsRobotInsideSections(sections=[ArenaSections.LEFT_GOAL, 
                                                      ArenaSections.RIGHT_GOAL]))
        
        self._move_to_point_task = GoToPosition()
        tree.add_child(self._move_to_point_task)

        return tree


    def ball_on_goalkeeper_section_tree(self) -> TreeNode:

        speed = 100
        accept_radius = AcceptanceRadiusEnum.LARGE.value


        # tree = Sequence("Ball on goakeeper section")

        # tree.add_child(IsBallInGoalkeeperSection())                           
        # tree.add_child(self._freeball_pos_task)


        tree = Sequence("Ball on goakeeper section")
        tree.add_child(IsBallInGoalkeeperSection())   

        enter_in_formation = Selector("Enter in formation")
        tree.add_child(enter_in_formation)

        # Gambiarra: Um usa univector outro não. Dois usando univector fica muito emocionante

        nearest_to_ball = Sequence("Nearest to ball action")
        nearest_to_ball.add_child(AmIClosestToBall())
        self._freeball_pos_task = GoToPosition(max_speed=speed, acceptance_radius=accept_radius)                                                   
        nearest_to_ball.add_child(self._freeball_pos_task)

        enter_in_formation.add_child(nearest_to_ball)

        self._middle_pos_task = GoToPosition(max_speed=speed, acceptance_radius=AcceptanceRadiusEnum.DEFAULT.value)

        enter_in_formation.add_child(self._middle_pos_task)


        return tree
        
    # Waits for the goalkeeper to get the ball out of the goalkeeper's section
    def _prepare_positions(self, blackboard: BlackBoard) -> None:


        team_side = blackboard.home_goal.side

        ball_pos = blackboard.ball.position
        
        # Gambiarra: se o robo tiver na seção de cima, vai para o free ball de cima
        if ball_pos[1] > 65:
            wait_position = Vec2D(75 + (-1 + 2*team_side) * 51, 107)
        else:
            wait_position = Vec2D(75 + (-1 + 2*team_side) * 51, 22)
        
        middle_point = Vec2D(75 + (-1 + 2*team_side) * 37, 65)
        # rospy.logfatal(f"{wait_position}")
        self._freeball_pos_task.set_position(wait_position)
        self._middle_pos_task.set_position(middle_point)
        
        ######################################################
        x_pos = 15 if team_side == 0 else 135
        self._move_to_point_task.target_pos = Vec2D(x_pos, 65)

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
 
        self._prepare_positions(blackboard)
        
        status, action = super().run(blackboard)
        return status, action

class FollowMainAttacker(TreeNode):

    def __init__(self, max_speed: int = 60, 
                       acceptance_radius: int = 20,
                       name: str = "FollowMainAttacker"):

        super().__init__(name)
        # Usando move2point
        self._follow_ally_task = FollowAlly(
            ally_id=1,
            max_speed=max_speed,
            acceptance_radius=acceptance_radius
        )


    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:

        id_to_follow = 2 if blackboard.robot.id == 1 else 1
        self._follow_ally_task.ally_id = id_to_follow
        
        return self._follow_ally_task.run(blackboard)


class AmIClosestToBall(TreeNode):
    def __init__(self, name: str = "AmIClosestBall"):
        super().__init__(name)

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:

        ball_position = blackboard.ball.position
        current_robot_to_ball_distance = (blackboard.robot.position - ball_position).norm()

        for robot in blackboard.home_team.robots:
            # Gambiarra: desconsidera robôs não inicializados e o goleiro
            if robot.id <= 0:
                continue
            robot_to_ball_distance = (robot.position - ball_position).norm()
            if robot_to_ball_distance < current_robot_to_ball_distance:
                return TaskStatus.FAILURE, NO_ACTION

        return TaskStatus.SUCCESS, NO_ACTION
