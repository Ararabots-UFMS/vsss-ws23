import rclpy
from rclpy.node import Node
import numpy as np
from struct import unpack
from utils.linalg import Vec2D
from utils.ros_utils import MsgOrigin
from sys_interfaces.msg import ThingsPosition, GameTopic, DebugTopic
from strategy.strategy_utils import GameStates
from strategy.behaviour import Goal


class RosRobotSubscriberAndPublisher:
    """
    This class is responsible for reading and formatting ros messages for the robot node
    """

    def __init__(self, _node: Node, _robot, _game_topic_name='game_topic_0', _should_debug=False):
        """
        :param _robot: robot object
        """
        #TODO: trocar uso do '_'. e.g: self.node -> self._node
        # self.node = _node
        self.robot = _robot

        self.robot.create_subscription(
            ThingsPosition, 
            '/things_position',
            self.read_topic, 
            qos_profile=5
        )

        self.robot.create_subscription(
            GameTopic, 
            _game_topic_name, 
            self.read_game_topic, 
            qos_profile=5
        )

        if int(_should_debug):            
            self.pub = self.robot.create_publisher(
                DebugTopic, 
                'debug_topic', 
                qos_profile=1
            )


        self.debug_msg = DebugTopic()
        self.debug_msg.id = self.robot.id
        self.read_game_topic(GameTopic())

    def read_game_topic(self, data : GameTopic):
        """
        Read from game topic callback and open the message into robot variables
        :param data: ROS game topic message
        :return: nothing
        """
        self.robot.blackboard.game.state = GameStates(data.game_state)
        
        if data.msg_origin == MsgOrigin.GAME_TOPIC.value:
            self.robot.blackboard.home_goal.side = data.team_side
            self.robot.blackboard.enemy_goal.side = not data.team_side
            self.robot.team_color = data.team_color
            self.robot.blackboard.robot.role = data.robot_roles[self.robot.id]
            self.robot.behaviour_tree = self.robot.behaviour_trees[self.robot.blackboard.robot.role]
            self.robot.blackboard.current_automatic_position = data.automatic_position


        self.robot.blackboard.game.penalty_robot_id = data.penalty_robot
        self.robot.blackboard.game.freeball_robot_id = data.freeball_robot
        self.robot.blackboard.game.meta_robot_id = data.meta_robot


    def read_topic(self, data : ThingsPosition) -> None:
        """
        This class formats the things position into np arrays and replaces any nan to None
        :param data: ROS Things position message
        :return: nothing
        """
        self.robot.blackboard.ball.position = Vec2D.from_array(np.array(data.ball_pos) / 100.0)

        if self.robot.team_color == 1:  # yellow
            friends_ids = np.array(data.yellow_team_ids)
            friends_speed = np.array(data.yellow_team_speed).reshape((-1, 2))
            friends_position = np.array(data.yellow_team_pos).reshape((-1, 2)) / 100.0
            friends_orientation = np.array(data.yellow_team_orientation) / 10000.0
            friends_vorientation = np.array(data.yellow_team_vorientation)

            enemies_ids = np.array(data.blue_team_ids)
            enemies_speed = np.array(data.blue_team_speed).reshape((-1, 2))
            enemies_position = np.array(data.blue_team_pos).reshape((-1, 2)) / 100.0
            enemies_orientation = np.array(data.blue_team_orientation) / 10000.0
            enemies_vorientation = np.array(data.blue_team_vorientation)
        else:  # blue
            friends_ids = np.array(data.blue_team_ids)
            friends_speed = np.array(data.blue_team_speed).reshape((-1, 2))
            friends_position = np.array(data.blue_team_pos).reshape((-1, 2)) / 100.0
            friends_orientation = np.array(data.blue_team_orientation) / 10000.0
            friends_vorientation = np.array(data.blue_team_vorientation)

            enemies_ids = np.array(data.yellow_team_ids)
            enemies_speed = np.array(data.yellow_team_speed).reshape((-1, 2))
            enemies_position = np.array(data.yellow_team_pos).reshape((-1, 2)) / 100.0
            enemies_orientation = np.array(data.yellow_team_orientation) / 10000.0
            enemies_vorientation = np.array(data.yellow_team_vorientation)

        # TODO: Refatorar aqui para incluir novas informações do things_position
        self.robot.blackboard.set_robot_variables(friends_ids[self.robot.tag],
                                                  friends_position[self.robot.tag],
                                                  friends_orientation[self.robot.tag],
                                                  friends_speed[self.robot.tag],
                                                  friends_vorientation[self.robot.tag])

        self.robot.blackboard.home_team.set_team_variables(friends_ids,
                                                           friends_position,
                                                           friends_orientation, 
                                                           friends_speed,
                                                           friends_vorientation,
                                                           robot_tag_index=self.robot.tag)

        self.robot.blackboard.enemy_team.set_team_variables(enemies_ids,
                                                            enemies_position,
                                                            enemies_orientation,
                                                            enemies_speed,
                                                            enemies_vorientation)
        self.robot.run()

    def debug_publish(self, _vector):

        """
            This function publishes in the debug topic
            :param vector: float64[2]
            :return: returns nothing
        """

        self.debug_msg.vector = _vector

        try:
            self.pub.publish(self.debug_msg)
        except Exception as e:
            self.robot.get_logger().fatal(e)
