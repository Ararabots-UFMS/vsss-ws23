import math
from abc import ABC, abstractmethod
import profile
from typing import Iterable
from typing import Tuple
from utils.linalg import Vec2D
from copy import deepcopy
import time

import numpy as np
# import rospy

from strategy.acceptance_radius import AcceptanceRadiusEnum

from robot.movement.definitions import OpCodes
from robot.movement.univector.un_field import UnivectorField

from strategy.arena_utils import HALF_ARENA_HEIGHT, get_defense_range_height, LEFT_AREA_CENTER_X, RIGHT_AREA_CENTER_X,\
    ROBOT_SIZE, is_on_y_upper_half, RIGHT, HALF_ARENA_WIDTH, on_attack_side, univector_pos_section, ArenaSections
from strategy.behaviour import ACTION, TreeNode
from strategy.behaviour import TaskStatus, BlackBoard, NO_ACTION
from strategy.strategy_utils import spin_direction, object_in_defender_range
from utils.json_handler import JsonHandler
from utils.math_utils import predict_speed, angle_between, clamp

from utils.debug_profile import debug_profiler

LEFT, RIGHT = 0, 1

class UpdateOrientationStopAction(TreeNode):

    def __init__(self, name: str = 'Stop Task'):
        super().__init__(name)

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        
        direction = blackboard.ball.position - blackboard.robot.position
        theta = math.atan2(direction[1], direction[0])

        return TaskStatus.RUNNING, (OpCodes.STOP, theta, 0, .0)

class StopAction(TreeNode):

    def __init__(self, name: str = 'Stop Task'):
        super().__init__(name)

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        return TaskStatus.RUNNING, (OpCodes.STOP, .0, 0, .0)


class SpinTask(TreeNode):
    def __init__(self, name='Spin Task', invert=False):
        super().__init__(name)
        self.invert = invert

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        
        return TaskStatus.RUNNING, (spin_direction(blackboard.ball.position, blackboard.robot.position,
                                                   team_side=blackboard.home_goal.side, invert=self.invert), 0.0, 255,
                                    .0)


class UnivectorTask(ABC):
    def __init__(self, name: str,
                 max_speed: int = 250,
                 acceptance_radius: float = AcceptanceRadiusEnum.DEFAULT.value,
                 speed_prediction: bool = True):
        self.name = name
        self.speed = max_speed
        self.speed_prediction = speed_prediction
        self._acceptance_radius = acceptance_radius

        univector_list = JsonHandler().read("parameters/univector_constants.json")

        # univector
        RADIUS = univector_list['RADIUS']
        KR = univector_list['KR']
        K0 = univector_list['K0']
        DMIN = univector_list['DMIN']
        LDELTA = univector_list['LDELTA']

        self.univector_field = UnivectorField()
        self.univector_field.update_constants(RADIUS, KR, K0, DMIN, LDELTA)

    @abstractmethod
    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        raise Exception("subclass must override run method")

    def go_to_objective(self, blackboard: BlackBoard, objective_position):

        distance_to_ball = (blackboard.robot.position - objective_position).norm()

        if distance_to_ball < self._acceptance_radius:
            return TaskStatus.SUCCESS, (OpCodes.STOP, 0, 0, 0)

        # Precisa ser deepcopy? Não sei

        all_robots_positions = deepcopy(blackboard.enemy_team.positions)
        all_robots_positions.extend(blackboard.home_team.positions)

        # TODO: Considerar velocidades dos obstáculos
        self.univector_field.update_obstacles(all_robots_positions,
                                             [Vec2D.origin() for i in range(len(all_robots_positions))])  # blackboard.enemies_speed)
        theta = blackboard.robot.orientation
        vec = 4*(-1 + 2*blackboard.current_orientation)*Vec2D(math.cos(theta), math.sin(theta))
        angle = self.univector_field.get_angle_with_ball(blackboard.robot.position + vec, Vec2D.origin(),
                                                         # blackboard.speed,
                                                         objective_position, _attack_goal=blackboard.enemy_goal.side)
        speed = self.speed
        # if self.speed_prediction:
        #     raio = predict_speed(blackboard.robot.position,
        #                          [np.cos(blackboard.robot.orientation), np.sin(blackboard.robot.orientation)],
        #                          objective_position,
        #                          self.univector_field.get_attack_goal_axis(blackboard.enemy_goal.side))
        #     cte = 90
        #     speed = (raio * cte) ** 0.5 + 10

        status = TaskStatus.RUNNING

        return status, (OpCodes.SMOOTH, angle, speed, distance_to_ball)


class GoToPositionUsingUnivector(UnivectorTask):

    # Gambiarra: Essa disgraça é exatamente a mesma coisa que o GoToGoalUsingUnivector, mas sem tempo pra refatorar agora ta ok

    def __init__(self, name="Go to position Univector", max_speed: int = 75, 
                 acceptance_radius: float = AcceptanceRadiusEnum.DEFAULT.value,
                 speed_prediction: bool = False, position=None):
        super().__init__(name, max_speed, acceptance_radius, speed_prediction)
        self.position = Vec2D.from_array(position) if position is not None else Vec2D.origin()

    def set_position(self, new_pos):
        self.position = new_pos

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:

        distance_to_position = (self.position - blackboard.robot.position).norm()

        if distance_to_position < self._acceptance_radius:
            return TaskStatus.SUCCESS, (OpCodes.STOP, 0, 0, 0)

        self.univector_field.update_obstacles(blackboard.enemy_team.positions,
                                              [Vec2D.origin() for i in range(len(blackboard.enemy_team.positions))])  # blackboard.enemies_speed)

        axis = Vec2D(-1.0, 0.0) if self.position[0] < HALF_ARENA_WIDTH else Vec2D(1.0, 0.0)
        # rospy.logfatal(f"Minha max e: {self.speed}")
        angle = self.univector_field.get_angle_vec(blackboard.robot.position, blackboard.robot.speed, # neste lugar estava robot.orientation
                                                   self.position, axis)

        status = TaskStatus.RUNNING

        return status, (OpCodes.SMOOTH, angle, self.speed, distance_to_position)


class RecoverBallUsingUnivector(UnivectorTask):
    def __init__(self, name: str = "Recover Ball Using Univector", max_speed: int = 250, 
    acceptance_radius: float = AcceptanceRadiusEnum.DEFAULT.value,
                 speed_prediction: bool = True):
        super().__init__(name, max_speed, acceptance_radius, speed_prediction)

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        distance_to_ball = (blackboard.robot.position - blackboard.ball.position).norm()

        if distance_to_ball < self._acceptance_radius:
            return TaskStatus.SUCCESS, (OpCodes.STOP, 0, 0, 0)

        # TODO: Considerar velocidades dos obstáculos
        self.univector_field.update_obstacles(blackboard.enemy_team.positions,
                                              [Vec2D.origin() for i in range(len(blackboard.enemy_team.positions))])  # blackboard.enemies_speed)

        axis = Vec2D(0.0, 1.0) if blackboard.ball.position[1] > HALF_ARENA_HEIGHT else Vec2D(0.0, -1.0)

        angle = self.univector_field.get_angle_vec(blackboard.robot.position, blackboard.robot.speed, # neste lugar estava robot.orientation
                                                   blackboard.ball.position, axis)

        status = TaskStatus.RUNNING

        return status, (OpCodes.SMOOTH, angle, self.speed, distance_to_ball)


class GoToBallUsingUnivector(UnivectorTask):

    def __init__(self, name: str = "Follow Ball", max_speed: int = 250, 
                 acceptance_radius: float = AcceptanceRadiusEnum.DEFAULT.value,
                 speed_prediction: bool = True):
        super().__init__(name, max_speed, acceptance_radius, speed_prediction)

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        # TODO: Profile
        # debug_profiler.enable()
        if self.speed_prediction:
            aux = self.go_to_objective(blackboard, blackboard.ball.position_prediction(1.3))
        else:
            aux = self.go_to_objective(blackboard, blackboard.ball.position)
        # aux = self.go_to_objective(blackboard, blackboard.ball.position)
        # debug_profiler.disable()

        return aux


class GoToAttackGoalUsingUnivector(UnivectorTask):

    def __init__(self, name, max_speed: int = 250, 
                 acceptance_radius: float = AcceptanceRadiusEnum.DEFAULT.value, 
    speed_prediction: bool = True):
        super().__init__(name, max_speed, acceptance_radius, speed_prediction)

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        return self.go_to_objective(blackboard, blackboard.enemy_goal.position)


class ChargeWithBall(TreeNode):
    def __init__(self, name='ChargeWithBall', max_speed: int = 255):
        super().__init__(name)
        self.max_speed = max_speed
        self.x_vector = Vec2D(1.0, 0.0)

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        goal_vector = blackboard.enemy_goal.position - blackboard.robot.position

        angle = angle_between(
            self.x_vector,
            goal_vector,
            absol=False
        )

        distance_to_goal = goal_vector.norm()

        return TaskStatus.RUNNING, (OpCodes.NORMAL, angle, self.max_speed, distance_to_goal)


class RemoveBallFromGoalArea(TreeNode):
    def __init__(self, name='RemoveBallFromGoalArea', max_speed: int = 255):
        super().__init__(name)
        self.max_speed = max_speed
        self.x_vector = Vec2D(1.0, 0.0)

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        goal_vector = blackboard.ball.position - blackboard.robot.position

        angle = angle_between(
            self.x_vector,
            goal_vector,
            absol=False
        )

        distance_to_goal = goal_vector.norm()
        # rospy.logfatal("Atrás da bola")
        return TaskStatus.RUNNING, (OpCodes.NORMAL, angle, self.max_speed, distance_to_goal)


class MarkBallOnAxis(TreeNode):
    def __init__(self, name: str = "MarkBallOnAxis",
                 max_speed: int = 255,
                 axis: np.ndarray = Vec2D(.0, 1.0),
                 acceptance_radius: float = AcceptanceRadiusEnum.DEFAULT.value,
                 clamp_min: float = None,
                 clamp_max: float = None,
                 predict_ball: bool = False
                 ):
        super().__init__(name)
        self._acceptance_radius = acceptance_radius
        self._max_speed = max_speed
        self._angle_to_correct = angle_between(Vec2D.right(), axis)
        self.turn_off_clamp = clamp_min is None and clamp_max is None
        self._clamp_min = clamp_min
        self._clamp_max = clamp_max
        self._predict_ball = predict_ball

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:

        if self._predict_ball:
            t = blackboard.ball.get_time_on_axis(axis=0, value=blackboard.robot.position[0])
            predicted_position = blackboard.ball.position_prediction(t)
            if abs(predicted_position[1] - blackboard.robot.position[1]) > self._acceptance_radius:
                target_position = predicted_position
            else:
                target_position = blackboard.ball.position
        else:
            target_position = blackboard.ball.position

        if self.turn_off_clamp:
            direction = target_position[1] - blackboard.robot.position[1]
        else:
            direction = clamp(target_position[1], self._clamp_min, self._clamp_max) - \
                        blackboard.robot.position[1]

        distance = abs(direction)

        if distance < self._acceptance_radius:
            return TaskStatus.RUNNING, (OpCodes.NORMAL,
                                        -self._angle_to_correct if direction < 0 else self._angle_to_correct, 0,
                                        distance)

        return TaskStatus.RUNNING, (OpCodes.NORMAL,
                                    -self._angle_to_correct if direction < 0 else self._angle_to_correct,
                                    self._max_speed,
                                    .0)


class MarkBallOnYAxis(TreeNode):
    def __init__(self, clamp_min: Iterable,
                 clamp_max: Iterable,
                 max_speed: int = 255,
                 name: str = "MarkBallOnYAxis",
                 acceptance_radius: float = AcceptanceRadiusEnum.DEFAULT.value):
        super().__init__(name)
        self._acceptance_radius = acceptance_radius
        self._max_speed = max_speed

        self._clamp_min = np.array(clamp_min)
        self._clamp_max = np.array(clamp_max)

    def set_clamps(self, clamp_min: Iterable, clamp_max: Iterable) -> None:
        self._clamp_min = clamp_min
        self._clamp_max = clamp_max

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        norm_distance = abs(blackboard.ball.position[0] - HALF_ARENA_WIDTH) / HALF_ARENA_WIDTH

        if norm_distance > 0.6:
            ball_y = blackboard.ball.position[1]
        else:
            # t = blackboard.ball.get_time_on_axis(axis=0, value=blackboard.home_goal.position[0])
            # TODO: média movel
            ball_y = blackboard.ball.position_prediction(1.3)[1]

        
        y = clamp(ball_y, self._clamp_min[1], self._clamp_max[1])

        target_pos = Vec2D(self._clamp_min[0], y)
        direction = target_pos - blackboard.robot.position
        distance = direction.norm()

        if distance < self._acceptance_radius:
            return TaskStatus.SUCCESS, NO_ACTION

        direction /= distance

        def gaussian(m, v):
            return math.exp(-(m ** 2) / (2 * (v ** 2)))

        alpha = gaussian(distance - self._acceptance_radius, 4.5)

        y_sign = -1 if direction[1] < 0 else 1
        direction_on_target = Vec2D(0, y_sign)

        final_direction = alpha * direction_on_target + (1 - alpha) * direction
        theta = math.atan2(final_direction[1], final_direction[0])

        return TaskStatus.RUNNING, (OpCodes.SMOOTH, theta, self._max_speed, distance)


class AlignWithAxis(TreeNode):
    def __init__(self, name: str = "AlignWithAxis",
                 max_speed: int = 0,
                 axis: Vec2D = Vec2D(.0, 1.0),
                 acceptance_angle: float = 0.0872665, align_with_ball: bool = False):

        super().__init__(name)
        self.max_speed = max_speed
        self._acceptance_angle = acceptance_angle
        self.angle_to_correct = angle_between(Vec2D.left(), axis)

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        if abs(self.angle_to_correct - abs(blackboard.robot.orientation)) <= self._acceptance_angle:
            status = TaskStatus.SUCCESS
        else:
            status = TaskStatus.RUNNING
        return status, (OpCodes.SMOOTH + OpCodes.ORIENTATION_AVERAGE, self.angle_to_correct, self.max_speed, .0)


class GetOutOfGoal(TreeNode):

    def __init__(self, name: str = "Go to goal area center",
                 max_speed: int = 255,
                 acceptance_radius: float = AcceptanceRadiusEnum.DEFAULT.value,
                 target_pos: list = None):
        super().__init__(name)
        self._acceptance_radius = acceptance_radius
        self.max_speed = max_speed
        self.target_pos = target_pos
    
    def _get_robot_center_shift(self, robot_pos):
        if is_on_y_upper_half(robot_pos):
            return -ROBOT_SIZE
        else:
            return ROBOT_SIZE

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        team_goal_side = blackboard.home_goal.side
        robot_pos = blackboard.robot.position
        new_y_pos = 0

        if self.target_pos is None:
            robot_center_shift = self._get_robot_center_shift(robot_pos)

            new_y_pos = robot_pos[1] + robot_center_shift
            if team_goal_side:
                self.target_pos = Vec2D(RIGHT_AREA_CENTER_X, new_y_pos)
            else:
                self.target_pos = Vec2D(LEFT_AREA_CENTER_X, new_y_pos)
        
        path = self.target_pos - robot_pos

        distance = path.norm()
        theta = math.atan2(path[1], path[0])

        if distance <= self._acceptance_radius:
            self.target_pos = None

            return TaskStatus.SUCCESS, (OpCodes.STOP, 0, 0, .0)

        return TaskStatus.RUNNING, (OpCodes.SMOOTH, theta, self.max_speed, distance)


class MoveBackward(TreeNode):

    def __init__(self, name: str = "MoveBackward",
                 max_speed: int = 255,
                 acceptance_radius: float = AcceptanceRadiusEnum.DEFAULT.value,
                 distance: float = 0.0):
        super().__init__(name)
        self._acceptance_radius = acceptance_radius
        self.max_speed = max_speed
        self.distance = distance

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        orientation = blackboard.robot.orientation

        return TaskStatus.RUNNING, (OpCodes.SMOOTH, -orientation, self.max_speed, self.distance)


class GoToGoalCenter(TreeNode):

    def __init__(self, name: str = "GoToGoalCenter",
                 max_speed: int = 255,
                 acceptance_radius: float = AcceptanceRadiusEnum.DEFAULT.value):
        super().__init__(name)
        self._acceptance_radius = acceptance_radius
        self.max_speed = max_speed

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        home_goal_pos = blackboard.home_goal.position
        if blackboard.home_goal.side == RIGHT:
            home_goal_pos[0] = RIGHT_AREA_CENTER_X
        else:
            home_goal_pos[0] = LEFT_AREA_CENTER_X

        direction = home_goal_pos - blackboard.robot.position

        distance = direction.norm()

        if distance < self._acceptance_radius:
            return TaskStatus.SUCCESS, (OpCodes.SMOOTH, 0, 0, distance)

        theta = math.atan2(direction[1], direction[0])
        return TaskStatus.RUNNING, (OpCodes.SMOOTH, theta, self.max_speed, distance)



class GoToGoalCenterUsingUnivector(UnivectorTask):

    def __init__(self, name: str = "GoToGoalCenter",
                 max_speed: int = 255,
                 acceptance_radius: float = AcceptanceRadiusEnum.DEFAULT.value):
        super().__init__(name, max_speed, acceptance_radius)
        self._acceptance_radius = acceptance_radius
        self.max_speed = max_speed
        self.x_offset = 2

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        # Ajustes na posição go gol
        home_goal_pos = blackboard.home_goal.position
        if blackboard.home_goal.side == RIGHT:
            home_goal_pos[0] = RIGHT_AREA_CENTER_X - self.x_offset
        else:
            home_goal_pos[0] = LEFT_AREA_CENTER_X + self.x_offset
        
        distance_to_goal = (home_goal_pos- blackboard.robot.position).norm()

        if distance_to_goal < self._acceptance_radius:
            return TaskStatus.SUCCESS, (OpCodes.STOP, 0, 0, 0)

        self.univector_field.update_obstacles(blackboard.enemy_team.positions,
                                              [Vec2D.origin() for i in range(len(blackboard.enemy_team.positions))])  # blackboard.enemies_speed)

        axis = Vec2D(-1.0, 0.0) if home_goal_pos[0] < HALF_ARENA_WIDTH else Vec2D(1.0, 0.0)

        angle = self.univector_field.get_angle_vec(blackboard.robot.position, blackboard.robot.speed, # neste lugar estava robot.orientation
                                                   home_goal_pos, axis)

        status = TaskStatus.RUNNING

        return status, (OpCodes.SMOOTH, angle, self.speed, distance_to_goal)


class GoToPosition(TreeNode):

    def __init__(self, name: str = "Straight Line Movement",
                 max_speed: int = 255,
                 acceptance_radius: float = AcceptanceRadiusEnum.DEFAULT.value,
                 target_pos: list = Vec2D.origin()):
        super().__init__(name)
        self._acceptance_radius = acceptance_radius
        self.max_speed = max_speed
        self.target_pos = Vec2D.from_array(target_pos) 

    def set_position(self, new_pos):
        self.target_pos = Vec2D.from_array(new_pos)

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        path = self.target_pos - blackboard.robot.position
        distance = path.norm()
        theta = math.atan2(path[1], path[0])

        if distance <= self._acceptance_radius:
            return TaskStatus.SUCCESS, (OpCodes.SMOOTH, 0, 0, .0)

        return TaskStatus.RUNNING, (OpCodes.SMOOTH, theta, self.max_speed, distance)

# TODO: Verificar o que a classe faz
class GoToDefenseRange(TreeNode):
    def __init__(self, name: str = "GoToDefenseRange", speed: int = 100, blackboard_key: str = 'ball'):
        super().__init__(name)
        self.speed = speed
        self.key = blackboard_key
        self.mark_points = (
            Vec2D(30, 25),
            Vec2D(30, 65),
            Vec2D(30, 105)
        )
        if blackboard_key == 'robot':
            self.verify_function = self.check_robot_x
        else:
            self.verify_function = self.check_ball_x

    def check_ball_x(self, blackboard: BlackBoard):
        return True

    def check_robot_x(self, blackboard: BlackBoard):
        robot_in_range = object_in_defender_range(blackboard.robot.position, blackboard.home_goal.side)
        if blackboard.home_goal.side == RIGHT:
            robot_in_home_side = blackboard.robot.position[0] >= 65
        else:
            robot_in_home_side = blackboard.robot.position[0] <= 85

        return robot_in_range and robot_in_home_side

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        objective_section = get_defense_range_height(blackboard.__getattribute__(self.key).position[1])
        robot_section = get_defense_range_height(blackboard.robot.position[1])

        if objective_section == robot_section and self.verify_function(blackboard):
            return TaskStatus.SUCCESS, NO_ACTION

        self.mark_points[objective_section][0] = 30 + blackboard.home_goal.side*90

        direction = self.mark_points[objective_section] - blackboard.robot.position
        distance = direction.norm()
        theta = math.atan2(direction[1], direction[0])

        return TaskStatus.RUNNING, (OpCodes.NORMAL, theta, self.speed, distance)


class GoToBallUsingMove2Point(TreeNode):
    def __init__(self, name: str = "GoToBallUsingMove2Point", speed=100, 
                 acceptance_radius: float = AcceptanceRadiusEnum.DEFAULT.value,
                 speed_prediction: bool = False):
        super().__init__(name)
        self.speed = speed
        self._acceptance_radius = acceptance_radius
        self._speed_prediction = speed_prediction

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        ball_section = univector_pos_section(blackboard.ball.position)

        ball_pos = blackboard.ball.position if not self._speed_prediction else blackboard.ball.position_prediction(1.3)

        direction = ball_pos - blackboard.robot.position

        distance = direction.norm()
        theta = math.atan2(direction[1], direction[0])
        if distance < self._acceptance_radius:
            return TaskStatus.SUCCESS, (OpCodes.NORMAL, 0.0, 0, 0)

        return TaskStatus.RUNNING, (OpCodes.NORMAL, theta, self.speed, distance)


class GoBack(TreeNode):
    def __init__(self, name: str = "GoBack",
                 max_speed: int = 80,
                 acceptance_radius: float = AcceptanceRadiusEnum.DEFAULT.value):
        super().__init__(name)
        self._acceptance_radius = acceptance_radius
        self.max_speed = max_speed

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        target_position = Vec2D(75., blackboard.robot.position[1])
        path = target_position - blackboard.robot.position
        distance = path.norm()
        theta = math.atan2(path[1], path[0])

        if distance <= self._acceptance_radius:
            return TaskStatus.SUCCESS, (OpCodes.NORMAL, 0, 0, .0)

        return TaskStatus.RUNNING, (OpCodes.NORMAL, theta, self.max_speed, distance)

class CanUseMoveToPointSafely(TreeNode):
    def __init__(self, name: str = "CanRobotUseMoveToPointSafely"):
        super().__init__(name)
    
    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        team_side = blackboard.home_goal.side
        robot_x = blackboard.robot.position[0]
        ball_x = blackboard.ball.position[0]
        
        if team_side == LEFT:
            if robot_x < ball_x:
                return TaskStatus.SUCCESS, NO_ACTION
            else:
                return TaskStatus.FAILURE, NO_ACTION
        else:
            if robot_x > ball_x:
                return TaskStatus.SUCCESS, NO_ACTION
            else:
                return TaskStatus.FAILURE, NO_ACTION

class FollowAlly(TreeNode):

    def __init__(self, ally_id: int, 
                       max_speed: int = 60, 
                       acceptance_radius: int = 20,
                       name: str = "FollowAlly"):

        super().__init__(name)

        self.ally_id = ally_id
        # Usando move2point
        self._move_to_ally_task = GoToPosition(
                max_speed=max_speed, 
                acceptance_radius=acceptance_radius
        )
    
    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:

        ally_position = Vec2D.origin()
        # Supor que o aliado SEMPRE exista
        for robot in blackboard.home_team.robots:
            if robot.id == self.ally_id: ally_position = robot.position
        
        self._move_to_ally_task.set_position(ally_position)

        return self._move_to_ally_task.run(blackboard)


class Sleep(TreeNode):
    def __init__(self, timer):
        self.timer = timer

    def run(self):
        time.sleep(self.timer)
        return TaskStatus.SUCCESS, NO_ACTION

class AutomaticMove(TreeNode):
    def __init__(self, state, team_side, auto_pos_path='/home/vsss/vsss_ws/src/parameters/replacer_positions.json'):
        self.state = state
        self.team_side = team_side
        self.auto_pos_path = auto_pos_path

    def get_position(self):
        position_dict = JsonHandler.read(self.auto_pos_path)
        position_dict[self.state][self.team_role]

    # def run(self):