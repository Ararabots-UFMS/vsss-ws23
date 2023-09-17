import imp
import rclpy
from rclpy.node import Node
from queue import Queue
from collections import deque, namedtuple
import numpy as np
from numpy import array, asarray, any
from sys_interfaces.msg import ThingsPosition, DebugTopic
#from interface.View.MainWindowView import MainWindowView
from rclpy.qos import QoSPresetProfiles

class RosMainWindowSubscriber:
    def __init__(self, node:Node, window, game_topic_name = 'game_topic_0'):
        """
        This is class is mainly responsible for the ROS functions of Main Window View
        :return: nothing
        """
        self._node = node
        self._my_window = window
        #if isnode:
        #    rospy.init_node('virtual_field', anonymous=True)

        # Ros node for reading the buffer
        self._node.create_subscription(
            ThingsPosition, 
            '/things_position', 
            self.read, 
            qos_profile=QoSPresetProfiles.SENSOR_DATA.value)

        # Debug topic
        self._node.create_subscription(
            DebugTopic, 
            'debug_topic', 
            self.read_debug_topic, 
            qos_profile= QoSPresetProfiles.SENSOR_DATA.value)

        # Queue of data from Topic Things position
        msg = ThingsPosition()
        debug_msg = DebugTopic()

        self.data = deque([msg], maxlen = 5)  # Shapes the size of the Queue
        self.debug_data = deque([debug_msg], maxlen = 10)

    def read_debug_topic(self, debug_data):
        """
        Read from topic callback and appends to data buffer
        :return: nothing
        """
        # Inserts data in the Queue
        self.debug_data.append(debug_data)

    def read(self, data):
        """
        Read from topic callback and appends to data buffer
        :return: nothing
        """
        # Inserts data in the Queue
        self.data.append(data)

    def pop_item_debug(self):
        """
        Grabs an message from the queue and returns it in np.array format
        :return data_item : things position message
        """
        debug_item = None

        debug_item = [-1,-1]
        
        try:
            debug_item = self.debug_data.popleft()
            debug_item = [debug_item.id, np.nan_to_num(array(debug_item.vector))]
        
        except IndexError:
            pass
            #self._node.get_logger().info("vazia")

        return debug_item


    def pop_item(self):
        """
        Grabs an message from the queue and returns it in np.array format
        :return data_item : things position message
        """
        data_item = None

        try:
            topic_info = self.data.popleft()
            data_item = namedtuple('DataItem', ['ball_pos', 'team_pos', 
                                   'team_orientation', 'team_speed',
                                   'enemies_pos', 'enemies_orientation',
                                   'enemies_speed', 'vision_fps'])
            
            data_item.ball_pos = np.array(topic_info.ball_pos) / 100.0
            data_item.vision_fps = topic_info.vision_fps / 100.0

            if self._my_window.home_color == 1:
                data_item.team_pos = np.array(topic_info.yellow_team_pos).reshape((5, 2)) / 100.0
                data_item.team_orientation = np.array(topic_info.yellow_team_orientation) / 10000.0
                data_item.enemies_pos = np.array(topic_info.blue_team_pos).reshape((5, 2)) / 100.0
                data_item.enemies_orientation = np.array(topic_info.blue_team_orientation) / 10000.0
            else:
                data_item.team_pos = np.array(topic_info.blue_team_pos).reshape((5, 2)) / 100.0
                data_item.team_orientation = np.array(topic_info.blue_team_orientation) / 10000.0
                data_item.enemies_pos = np.array(topic_info.yellow_team_pos).reshape((5, 2)) / 100.0
                data_item.enemies_orientation = np.array(topic_info.yellow_team_orientation) / 10000.0


            enemies_position = []
            enemies_orientation = []

            for i in range(5):
                if any(data_item.enemies_pos[i]):
                    enemies_position.append(data_item.enemies_pos[i])
                    enemies_orientation.append(data_item.enemies_orientation[i])

            data_item.enemies_pos = asarray(enemies_position)
            data_item.enemies_orientation = asarray(enemies_orientation)


        except IndexError:
            pass
            # self._node.get_logger().info("vazia")

        return data_item
