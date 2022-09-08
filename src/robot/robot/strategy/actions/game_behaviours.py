from math import sin
from typing import Callable, List

from robot.strategy import arena_utils
from robot.strategy.arena_utils import on_attack_side, section, LEFT, HALF_ARENA_WIDTH, ArenaSections, is_on_y_upper_half, ROBOT_SIZE
from robot.strategy.behaviour import *
from robot.strategy.behaviour import ACTION, NO_ACTION, TreeNode
from robot.strategy.behaviour import BlackBoard, TaskStatus
from robot.strategy.strategy_utils import *
from utils.math_utils import angle_between

from robot.strategy.acceptance_radius import AcceptanceRadiusEnum


class IsBehindBall(TreeNode):
    def __init__(self, name: str, distance: int):
        super().__init__(name)
        self.distance = distance

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        if is_behind_ball(blackboard.ball.position,
                          blackboard.robot,
                          blackboard.home_goal.side,
                          self.distance,
                          max_angle=43):
            
            return TaskStatus.SUCCESS, NO_ACTION
        else:
            return TaskStatus.FAILURE, NO_ACTION


class IsTheWayFree(TreeNode):
    def __init__(self, name: str, free_way_distance: int):
        super().__init__(name)
        self.free_way_distance = free_way_distance

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        enemy_goal_pos = blackboard.enemy_goal.position
        ball_pos = blackboard.ball.position
        v_ball_enemy_goal = enemy_goal_pos - ball_pos

        task_result = TaskStatus.SUCCESS, NO_ACTION

        for enemy_position in blackboard.enemy_team.positions:
            enemy_goal = blackboard.enemy_goal.side
            v_ball_enemy = enemy_position - blackboard.ball.position
            theta = angle_between(v_ball_enemy_goal, v_ball_enemy, absol=False)
            enemy_to_path_distance = v_ball_enemy.norm() * sin(theta)

            if arena_utils.section(enemy_position).value != enemy_goal:
                # É utilizado o x da bola, pois caso o adversário esteja entre
                # o robô e a bola, o univector trata a situação
                ball_x = blackboard.ball.position[0]
                enemy_x = enemy_position[0]

                # Evita que o caminho seja considerado obstruído por
                # adversários atrás do robô
                if enemy_goal:
                    is_enemy_in_way = enemy_x > ball_x
                else:
                    is_enemy_in_way = enemy_x < ball_x

                if abs(enemy_to_path_distance) <= self.free_way_distance and \
                        is_enemy_in_way:
                    task_result = TaskStatus.FAILURE, NO_ACTION
                    break  # Interrompe o loop para o primeiro robô no caminho.
        return task_result


class CanDefenderUseMove2PointToRecoverBall(TreeNode):
    def __init__(self, name="CanDefenderUseMove2PointToRecoverBall?"):
        super().__init__(name)

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        if blackboard.ball.position[1] > HALF_ARENA_WIDTH:  # Ball is on upper side
            if blackboard.ball.position[1] > blackboard.robot.position[1]:
                return TaskStatus.SUCCESS, NO_ACTION
        else:
            if blackboard.ball.position[1] < blackboard.robot.position[1]:
                return TaskStatus.SUCCESS, NO_ACTION

        return TaskStatus.FAILURE, NO_ACTION


class IsBallInsideCentralArea(TreeNode):
    def __init__(self, name: str, width: int = 110, height: int = 90):
        super().__init__(name)
        self._width = width
        self._height = height
        self._center = Vec2D(150, 130) / 2
        self._tl = self._br = None
        self.update_corners()

    def update_corners(self) -> None:
        vec = Vec2D(self._width, -self._height) / 2
        self._tl = self._center - vec
        self._br = self._center + vec

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        ball_pos = blackboard.ball.position
        if self._tl[0] <= ball_pos[0] < self._br[0] and \
                self._br[1] <= ball_pos[1] < self._tl[1]:
            return TaskStatus.SUCCESS, NO_ACTION
        else:
            return TaskStatus.FAILURE, NO_ACTION


class IsInAttackSide(TreeNode):
    def __init__(self, name: str, get_pos: Callable[[BlackBoard], Vec2D]):
        super().__init__(name)
        self._get_pos = get_pos

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        side = blackboard.enemy_goal.side
        x_obj = self._get_pos(blackboard)[0]

        if (x_obj > 75 and side == RIGHT) or (x_obj < 75 and side == LEFT):
            status = TaskStatus.SUCCESS
        else:
            status = TaskStatus.FAILURE
        return status, NO_ACTION


class AmIAttacking(TreeNode):
    def __init__(self, name: str = "AmIAttacking"):
        super().__init__(name)

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        if ball_on_attack_side(blackboard.ball.position, blackboard.home_goal.side):
            return TaskStatus.SUCCESS, NO_ACTION
        return TaskStatus.FAILURE, NO_ACTION


class IsBallInRangeOfDefense(TreeNode):
    def __init__(self, name: str = "IsBallInRangeOfDefense"):
        super().__init__(name)

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        if not ball_on_attack_side(blackboard.ball.position, blackboard.home_goal.side) and \
                object_in_defender_range(blackboard.ball.position, blackboard.home_goal.side):
            return TaskStatus.SUCCESS, NO_ACTION
        return TaskStatus.FAILURE, NO_ACTION


class IsRobotInRangeOfDefense(TreeNode):
    def __init__(self, name: str = "IsBallInRangeOfDefense"):
        super().__init__(name)

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        if not ball_on_attack_side(blackboard.robot.position, blackboard.home_goal.side) and \
                object_in_defender_range(blackboard.robot.position, blackboard.home_goal.side):
            return TaskStatus.SUCCESS, NO_ACTION
        return TaskStatus.FAILURE, NO_ACTION


class IsBallInGoalkeeperSection(TreeNode):
    def __init__(self, name: str = "BallInFirstQuarter"):
        super().__init__(name)
    
    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        team_side = blackboard.home_goal.side
        ball_x = blackboard.ball.position[0]

        if team_side == LEFT and ball_x < 20:
            return TaskStatus.SUCCESS, NO_ACTION
        elif team_side == RIGHT and ball_x > 130:
            return TaskStatus.SUCCESS, NO_ACTION
            
        return TaskStatus.FAILURE, NO_ACTION

        

class IsBallInCriticalPosition(TreeNode):
    def __init__(self, name: str = "IsBallInCriticalPosition"):
        super().__init__(name)

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        if object_on_critical_position(blackboard.ball.position, blackboard.home_goal.side):
            return TaskStatus.SUCCESS, NO_ACTION
        return TaskStatus.FAILURE, NO_ACTION


class IsEnemyInCriticalPosition(TreeNode):
    def __init__(self, name: str = "IsEnemyInCriticalPosition"):
        super().__init__(name)

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        for enemy_position in blackboard.enemy_team.positions:
            if object_on_critical_position(enemy_position, blackboard.home_goal.side):
                return TaskStatus.SUCCESS, NO_ACTION
        return TaskStatus.FAILURE, NO_ACTION


class IsEnemyInsideAreas(TreeNode):
    def __init__(self, name: str = "IsEnemyInsideAreas", areas: List = []):
        super().__init__(name)
        self._areas = areas

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        for enemy in blackboard.enemy_team:
            if section(enemy.position) in self._areas:
                return TaskStatus.SUCCESS, NO_ACTION

        return TaskStatus.FAILURE, NO_ACTION


class IsBallInsideSections(TreeNode):
    def __init__(self, name: str = "IsBallInsideSections", sections: List = []):
        super().__init__(name)
        self._sections = sections

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        if section(blackboard.ball.position) in self._sections:
            return TaskStatus.SUCCESS, NO_ACTION
        return TaskStatus.FAILURE, NO_ACTION

class IsRobotInsideSections(TreeNode):
    def __init__(self, name: str = "IsBallInsideSections", sections: List = []):
        super().__init__(name)
        self._sections = sections

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        if section(blackboard.robot.position) in self._sections:
            return TaskStatus.SUCCESS, NO_ACTION
        return TaskStatus.FAILURE, NO_ACTION

class IsBallInDefenseBorder(TreeNode):
    def __init__(self, name: str = "IsBallInDefenseBorder"):
        super().__init__(name)

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        if ball_on_defense_border(blackboard.ball.position, blackboard.home_goal.side):
            return TaskStatus.SUCCESS, NO_ACTION
        return TaskStatus.FAILURE, NO_ACTION


class IsBallInBorder(TreeNode):
    def __init__(self, name: str = "IsBallInBorder"):
        super().__init__(name)

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        if ball_on_border(blackboard.ball.position):
            return TaskStatus.SUCCESS, NO_ACTION
        return TaskStatus.FAILURE, NO_ACTION


class AmIInDefenseField(TreeNode):
    def __init__(self, name: str = "AmIInDefenseField"):
        super().__init__(name)

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        if not on_attack_side(blackboard.robot.position, blackboard.home_goal.side):
            return TaskStatus.SUCCESS, NO_ACTION

        return TaskStatus.FAILURE, NO_ACTION


class IsEnemyNearRobot(TreeNode):
    def __init__(self, name: str = "IsEnemyNearRobot", acceptance_radius=AcceptanceRadiusEnum.NORMAL.value):
        super().__init__(name)
        self._acceptance_radius = acceptance_radius

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        for enemy_position in blackboard.enemy_team.positions:
            if distance_point(blackboard.robot.position, enemy_position) <= self._acceptance_radius:
                return TaskStatus.SUCCESS, NO_ACTION

        return TaskStatus.FAILURE, NO_ACTION


class IsEnemyNearBall(TreeNode):
    def __init__(self, name: str = "IsEnemyNearBall", acceptance_radius=AcceptanceRadiusEnum.NORMAL.value):
        super().__init__(name)
        self._acceptance_radius = acceptance_radius

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        for enemy_position in blackboard.enemy_team.positions:
            if near_ball(blackboard.ball.position, enemy_position, self._acceptance_radius):
                return TaskStatus.SUCCESS, NO_ACTION

        return TaskStatus.FAILURE, NO_ACTION


class IsNearBall(TreeNode):
    def __init__(self, name: str = "IsNearBall", acceptance_radius=AcceptanceRadiusEnum.SMALL.value):
        super().__init__(name)
        self._acceptance_radius = acceptance_radius

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        if near_ball(blackboard.ball.position, blackboard.robot.position, self._acceptance_radius):
            return TaskStatus.SUCCESS, NO_ACTION
        else:
            return TaskStatus.FAILURE, NO_ACTION


class IsInsideMetaRange(TreeNode):
    def __init__(self, name: str, distance: int = 25):
        super().__init__(name)
        self.distance = distance

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        if abs(blackboard.robot.position[0] - blackboard.home_goal.position[0]) < self.distance:
            return TaskStatus.SUCCESS, NO_ACTION
        else:
            return TaskStatus.FAILURE, NO_ACTION


class IsInsideDefenseGoal(TreeNode):
    def __init__(self, name: str,
                 get_pos: Callable[[BlackBoard], Vec2D]):
        super().__init__(name)
        self._get_pos = get_pos

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        pos = self._get_pos(blackboard)
        team_side = blackboard.home_goal.side

        sign = 1 if team_side == RIGHT else -1

        shift = sign * 0 # int(ROBOT_SIZE/2)
        shifted_pos = Vec2D(pos[0] + shift, pos[1])
        section = arena_utils.section(shifted_pos)

        my_goal = ArenaSections.LEFT_GOAL if team_side == LEFT \
            else ArenaSections.RIGHT_GOAL
        if section == my_goal:
            return TaskStatus.SUCCESS, NO_ACTION
        else:
            return TaskStatus.FAILURE, NO_ACTION


class IsRobotInsideEnemyGoalLine(TreeNode):
    def run(self, blackboard: BlackBoard):
        robot_position = blackboard.robot.position
        ball_position = blackboard.ball.position
        enemy_goal_line_y = [30, 100]

        if (ball_position[1] <= enemy_goal_line_y[0] or
                ball_position[1] >= enemy_goal_line_y[1]):

            if (robot_position[1] <= enemy_goal_line_y[0] or
                    robot_position[1] >= enemy_goal_line_y[1]):

                if blackboard.enemy_goal.side == LEFT:
                    enemy_goal_line_x = arena_utils.LEFT_GOAL_LINE

                    if robot_position[0] <= enemy_goal_line_x and ball_position[0] <= enemy_goal_line_x:
                        return TaskStatus.SUCCESS, NO_ACTION

                elif blackboard.enemy_goal.side == RIGHT:
                    enemy_goal_line_x = arena_utils.RIGHT_GOAL_LINE

                    if robot_position[0] >= enemy_goal_line_x and ball_position[0] >= enemy_goal_line_x:
                        return TaskStatus.SUCCESS, NO_ACTION

        return TaskStatus.FAILURE, NO_ACTION


class IsInDefenseBottomLine(TreeNode):
    def __init__(self, name: str, get_pos: Callable[[BlackBoard], Vec2D]):
        super().__init__(name)
        self._get_pos = get_pos

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        side = blackboard.home_goal.side
        x_obj, y_obj = self._get_pos(blackboard)
        if (side == LEFT and x_obj > 20) or (side == RIGHT and x_obj < 130):
            return TaskStatus.FAILURE, NO_ACTION

        if y_obj < 30 or y_obj > 100:
            return TaskStatus.SUCCESS, NO_ACTION

        return TaskStatus.FAILURE, NO_ACTION


class CanAttackerUseMoveToPointToGuideBall(TreeNode):
    def __init__(self, name="CanAttackerUseMoveToPointToGuideBall?"):
        super().__init__(name)

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        ball_pos = blackboard.ball.position
        if is_on_y_upper_half(ball_pos):
            border_vec = Vec2D(1.0, 0.0)
        else:
            border_vec = Vec2D(-1.0, 0.0)
        robot_pos = blackboard.robot.position
        ball_pos = blackboard.ball.position
        robot_ball_vec = robot_pos - ball_pos

        theta = angle_between(robot_ball_vec,
                              border_vec,
                              absol=False)
                              
        if theta < math.pi/6:
            return TaskStatus.FAILURE, NO_ACTION
        return TaskStatus.SUCCESS, NO_ACTION
