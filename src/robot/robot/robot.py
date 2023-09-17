import time
from typing import List, Tuple

# import cv2
import rclpy
from rclpy.node import Node

from robot.ros_robot_subscriber_and_publiser import RosRobotSubscriberAndPublisher
# from interface.virtualField import virtualField, unit_convert, position_from_origin
from robot.comunication.sender import Sender, STDMsg, SelfControlMsg
from robot.control import Control
from robot.hardware import RobotHardware, SimulationHardware
from strategy.attacker import Attacker
from strategy.behaviour import BlackBoard, TaskStatus, OpCodes
from strategy.defender import Defender
from strategy.goalkeeper import GoalKeeper
from strategy.pid_calibration import CalibrationTree
from utils.json_handler import JsonHandler
from utils.linalg import *
from utils.watcher import watcher

class Robot(Node):

    def __init__(self, 
                 robot_id: int,
                 tag: int,
                 robot_body: str,
                 team_side: int,
                 team_color: int,
                 robot_role: int,
                 owner_name: str,
                 socket_id: int = -1,
                 socket_offset: int = 0,
                 should_debug: int = 0):

        robot_name = "ROBOT_" + str(robot_id)

        super().__init__(robot_name, namespace=owner_name)

        # Parameters
        self.id = robot_id
        
        self.robot_body = robot_body
        self.tag = tag
        self._socket_id = socket_id
        self._socket_offset = socket_offset
        self._should_debug = should_debug
        
        self._owner_name = owner_name        

        constants = self.get_pid_constants_set()
        self.get_logger().warn(str(constants))

        self._max_fine_movement_speed = 80
        self._hardware = RobotHardware()

        # TODO:Determinar qual velocidade máxima usar na simulação
        # TODO: Implementar condicao para simulador/hardware
        self._max_fine_movement_speed = 45
        self._hardware = SimulationHardware()


        self.blackboard = BlackBoard()
        self.blackboard.my_id = self.id
        self.blackboard.home_goal.side = team_side
        self.blackboard.enemy_goal.side = not team_side
        self.blackboard.robot.role = robot_role
        
        self._controller = Control(self._hardware, self.blackboard, constants, self._max_fine_movement_speed)
        self.pid_on_hardware = False

        # self.velocity_buffer = []
        # self.position_buffer = []

        # Receive from game topic
        self.team_color = team_color

        self.left_speed = self.right_speed = 0

        # Open bluetooth socket
        if self._socket_id == -1:
            self.get_logger().fatal("Using fake bluetooth")
            self._sender = None
        else:
            self._sender = Sender(self, self._socket_id, self._socket_offset, self._owner_name)

        # ROBOTO VISION
        #self.imgField = virtualField(4 * 150,
        #                             4 * 130)  
        # cv2.imread('src/robot_module/movement/univector/img/vss-field.jpg')

        self.behaviour_trees = [
            Attacker(),
            Defender(),
            GoalKeeper(),
            CalibrationTree()
        ]

        self.behaviour_tree = self.behaviour_trees[robot_role]
        self.stuck_counter = 0

        self.subsAndPubs = RosRobotSubscriberAndPublisher(self, self, 'game_topic',
                                                          self._should_debug)


    @property
    def position(self):
        return self.blackboard.robot.position

    @property
    def speed(self):
        return self.blackboard.robot.speed

    @property
    def orientation(self):
        return self.blackboard.robot.orientation

    @property
    def vorientation(self):
        return self.blackboard.robot.vorientation

    def get_pid_constants_set(self) -> List[Tuple]:
        pid_set = []
        bodies = JsonHandler.read("parameters/bodies.json", escape=True)
        try:
            pid_dict = bodies[self.robot_body]
        except KeyError:
            pid_dict = {
                "0": {
                    "KP": 0.0,
                    "KD": 0.0,
                    "KI": 0.0
                }
            }

        for speed in pid_dict:
            ctes = pid_dict[speed]
            pid_set.append((int(speed), ctes["KP"], ctes["KI"], ctes["KD"]))

        return pid_set

    def run(self):
        # try:
        task_status, action = self.behaviour_tree.run(self.blackboard)
        # except Exception as excp:
        #     self.get_logger().fatal(excp)
        #     raise excp

        # task_status = TaskStatus.FAILURE
        if task_status == TaskStatus.FAILURE or task_status is None:
            action = (OpCodes.STOP, 0.0, 0, 0)

        self._controller.update_orientation(self.orientation)

        if self.pid_on_hardware:
            direction, angle, speed = self._controller.get_correction_angle_and_speed(
                *action)
            msg = SelfControlMsg(direction, angle, speed)
        else:
            left, right = self._controller.get_wheels_speeds(*action)
            msg = STDMsg(left, right)
            msg = self._hardware.normalize_speeds(msg)
            
        # Print a cada mol segundos...
        # watcher.print(f"L: {msg[0]:.2f}, R: {msg[1]:.2f}", period=0.1)
        # watcher.print(f"vx: {self.speed[0]:.2f}, vy: {self.speed[1]:.2f}", period=0.2)
        # watcher.print(f"vorientation: {self.vorientation:.2f}", period=0.2)
        # watcher.print(f"{self.blackboard.enemy_team}", period=0.2)
            


        if self._sender is not None:
            priority = self.get_priority()
            # self.get_logger().fatal(str(msg))
            self._sender.send(priority, self._socket_id, self.team_color, self._hardware.encode(msg))

        # self.roboto_vision()

    def get_priority(self) -> int:
        distance = (self.blackboard.robot.position - self.blackboard.ball.position).norm()
        return int(distance) & 0xFF

    # def roboto_vision(self):
    #     self.imgField.plot_ball(self.blackboard.ball.position)
    #     t = self.blackboard.ball.get_time_on_axis(0, self.blackboard.ball.position[0])
    #     rospy.logfatal(t)
    #     ma_ball = self.blackboard.ball.position_prediction(t)

    #     ma_ball = unit_convert(ma_ball, self.imgField.width_conv, self.imgField.height_conv)
    #     ma_ball = position_from_origin(ma_ball, self.imgField.field_origin)
    #     rospy.logfatal(ma_ball)
    #     rospy.logfatal(self.blackboard.ball.position)
    #     # rospy.logwarn(ma_ball)
    #     cv2.circle(self.imgField.field, ma_ball, self.imgField.ball_radius,
    #                self.imgField.colors["red"], -1)
    #     cv2.imshow('Robot\'o Vision', self.imgField.field)
    #     cv2.waitKey(1)
