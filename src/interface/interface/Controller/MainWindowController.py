from typing import List, Tuple

import fltk as fl

from interface.View.MainWindowView import MainWindowView
from interface.Controller.BluetoothManagerController import BluetoothManagerController
from interface.Controller.ConnectionController import ConnectionController
from interface.Controller.DebugController import DebugController
from interface.Controller.RobotParamsController import RobotParamsController
from message_server.opcodes import ServerOpCode
from interface.ros_game_topic_publisher import GameTopicPublisher
from interface.main_window_controller_subscriber import MainWindowControllerSubscriber
from interface.Coach.Coach import Coach
from rclpy.node import Node
from threading import Thread

class MainWindowController:
    def __init__(self,node: Node, model: dict,
                 _coach: Coach,
                 _game_topic_publisher: GameTopicPublisher):
        """
        This class os resonsible for managing the main window
        """
        # Save the parameters for future use
        self.robot_params = model.robot_params
        self.robot_bluetooth = model.robot_bluetooth
        self.robot_roles = model.robot_roles
        self.game_opt = model.game_opt
        self.debug_params = model.debug_params

        # The controllers are created but not show
        self.bluetooth_controller = BluetoothManagerController(model, hidden=True)
        self.connection_controller = ConnectionController(self.game_opt)
        self.debug_controller = DebugController(model.debug_params, hidden=True)
        self.robot_params_controller = RobotParamsController(self.robot_params, self.robot_bluetooth,
                                                             self.robot_roles, model.robot_bodies)
# suffix = '' if owner_id is None else '_' + owner_id
        # Lets create the view of our controller shall we
        self._node = node
        self.view = MainWindowView(self._node, _game_topic_publisher.get_name())
        self.view.virtualField.set_univector_debug_params(not self.game_opt['side'],
                                                          model.debug_params['robot_vector'],
                                                          model.debug_params['things'],
                                                          self.robot_params)
        self.subs = MainWindowControllerSubscriber(self._node,self._connection_status_listener, _game_topic_publisher.get_owner())

        # Creates the game topic
        self.pub = _game_topic_publisher

        # The coach object class control the active and the activities of robots
        self.coach = _coach

        # Since our primary keys are the keys of the dict it self
        # and since our DataBase is simple, it can be stored as simple strings
        self.robot_roles_keys = sorted(self.robot_roles, key=self.robot_roles.get)  # sorted(self.robot_roles.keys())
        self.robot_bluetooth_keys = self.robot_bluetooth.keys()

        # Fast access array to use a dict as an simple array
        self.faster_hash = ['robot_' + str(x) for x in range(1, 6)]
        self.assigned_robot_text = ["Jogador " + str(x) for x in range(1, 6)]
        self.assigned_robot_indexes = ['penalty_player', 'freeball_player', 'meta_player']

        self.view.team_color.value(self.game_opt["time"])
        self.pub.set_team_color(self.view.team_color.value())

        self.set_robot_plot_color(self.game_opt["time"])

        self.view.team_side.value(self.game_opt["side"])

        # Replaced by the function set_robots_bluetooth
        self.set_robots_params()

        # A loop for the assigned robot actions
        for num in range(3):

            # Same idea here with the value indexes
            current_item = 0
            for item in self.assigned_robot_text:
                # Add the name of player to the drop-down...
                self.view.option_robots[num].add(item)

                # If robot name is equal to the game action, set as the current value
                if self.game_opt[self.assigned_robot_indexes[num]] == current_item:
                    # Same as above
                    self.view.option_robots[num].value(current_item)
                # Well...
                current_item += 1

            # Set a callback for input and button
            self.view.option_robots[num].callback(self.action_input_choice)
            self.view.action_buttons[num].callback(self.action_button_clicked)
        # Multiple callbacks, each for one type of input
        # but since whe have ids for each robot input
        # we can parse through each using its on dictionary
        for num in range(self.view.n_robots):
            self.view.robot_params[num].callback(self.parameters_button)
            self.view.robot_roles[num].callback(self.role_choice)
            self.view.robot_radio_button[num].callback(self.radio_choice)

        self.view.top_menu.callback(self.top_menu_choice)
        self.view.play_button.callback(self.action_button_clicked)

        self.view.team_color.callback(self.on_color_change)
        self.view.team_side.callback(self.on_side_change)

        # self._initialize_game_topic()
        #self.view.end()
        self.coach.ros_functions.spin_function(self.view.end)

    def _initialize_game_topic(self):
        self.pub.publish()

    def _connection_status_listener(self, topic: List) -> None:
        for i, status in enumerate(topic):
            ptr = self.view.robot_radio_button[i]
            if status != ptr.value():
                ptr.value(int(status))
                self.coach.set_robot_active(ptr.id, ptr.value())
                self.robot_params[self.faster_hash[ptr.id]]['active'] = ptr.value()

    def set_robots_params(self):
        """
        Set robot roles and active checks
        :return: nothing
        """
        # For each Robot, this loop covers all the inputs
        for num in range(self.view.n_robots):
            # Access the robot params dict using a int
            # and stores its reference in a temporary variable
            current_robot = self.robot_params[self.faster_hash[num]]

            # This integer is for the value of item in the
            # Drop-down choice box
            # This integer, again, is for the value of item in the
            # Drop-down choice box

            for item in self.robot_roles_keys:
                # Add the key of the dictionary to the drop-down...
                # again
                current_item = int(self.robot_roles[item])
                self.view.robot_roles[num].insert(current_item, item, 0, None, None, 0)

                if current_robot['role'] == item:
                    # Same as above
                    self.view.robot_roles[num].value(current_item)

            # The value for the check button
            self.view.robot_radio_button[num].value(current_robot['active'])
            # Unique id for the check-Box
            self.view.robot_radio_button[num].id = num

    def parameters_button(self, ptr):
        """
        Takes a point with id = robot_id, calls the Param Controller and compares old with new dict for
        assigning variables to the robot node
        :param ptr: Widget pointer
        :return: nothing
        """
        old_parameters = self.robot_params[self.faster_hash[ptr.id]].copy()

        self.robot_params_controller.show(ptr.id)
        while self.robot_params_controller.view.root.visible():
            fl.Fl.wait()

        new_parameters = self.robot_params[self.faster_hash[ptr.id]]

        the_same = new_parameters == old_parameters

        if not the_same:
            self.coach.set_robot_parameters(ptr.id)
            self.view.virtualField.set_visible_vectors(self.debug_params['things'], self.robot_params)
            self.coach.change_robot_tag(ptr.id, new_parameters['tag_number'])

    def top_menu_choice(self, ptr):
        """
        Callback function, using the value of pointer
        :param ptr: Widget pointer
        :return: nothing
        """
        if ptr.value() < 4:
            self.pub.send_vision_operation(ptr.value())

        if ptr.value() < 9:
            if ptr.value() == 7:  # Connection controller
                self.wait_window_close(self.connection_controller)
            if ptr.value() == 8:  # Bluetooth Controller
                self.wait_window_close(self.bluetooth_controller)

        elif ptr.value() < 16:
            if ptr.value() == 12:
                if self.view.virtualField.show_univector_tweaker:
                    self.view.virtualField.destroy_univector_trackbars()
                else:
                    self.view.virtualField.create_univector_trackbars()

            elif ptr.value() == 13:
                self.wait_window_close(self.debug_controller)
                self.view.virtualField.set_draw_vectors(self.debug_params['movement_predict_simulation'])
                self.view.virtualField.set_visible_vectors(self.debug_params['things'], self.robot_params)
            elif ptr.value() == 14:
                self.wait_window_close(self.connection_controller)

    def wait_window_close(self, window_controller):
        """
        Wait till window is not visible
        :param window_controller: Fl_Window
        :return: nothing
        """
        window_controller.show()
        while window_controller.view.root.visible():
            fl.Fl.wait()

    def action_input_choice(self, ptr):
        """
        Assign robot action callback
        :param ptr: Widget pointer
        :return: nothing
        """
        self.pub.assigned_action_to_robot(ptr.id, ptr.value())
        self.pub.publish()
        self.game_opt[self.assigned_robot_indexes[ptr.id]] = ptr.value()

    def get_mac_address(self, robot_id: int) -> Tuple[bool, bytes]:
        robot_name = self.robot_params[self.faster_hash[robot_id]]['bluetooth_mac_address']

        try:
            mac_str = self.robot_bluetooth[robot_name]
        except KeyError:
            mac_str = "-1"

        if mac_str == "-1":
            return False, bytes([])
        else:
            mac = [int(x, base=16) for x in mac_str.split(":")]
            return True, bytes(mac)

    def radio_choice(self, ptr):
        """
        Checkbox callback, node active or not?
        :param ptr: Widget pointer
        :return: nothing
        """        
        ptr.deactivate()
        thread = Thread(target=self.wait_message_server_response, args=(ptr,))
        thread.daemon = True
        thread.start()


    def wait_message_server_response(self, ptr):

        isMacAddress, mac = self.get_mac_address(ptr.id)

        if isMacAddress:
            op_code = ServerOpCode.ADD if ptr.value() else ServerOpCode.REMOVE
            r = self.pub.add_or_remove_socket_on_messageserver(op_code.value, ptr.id, mac)

        if not isMacAddress or r != ServerOpCode.ERROR.value:
            self.coach.set_robot_active(ptr.id, ptr.value())
            self.robot_params[self.faster_hash[ptr.id]]['active'] = ptr.value()
        
        ptr.activate()

    def role_choice(self, ptr):
        """
        Callback for robot role choice
        :param ptr: Widget pointer
        :return: nothing
        """
        self.coach.change_robot_role(ptr.id, ptr.value())
        self.robot_params[self.faster_hash[ptr.id]]['role'] = self.robot_roles_keys[ptr.value()]

    def action_button_clicked(self, ptr):
        """
        Action callback related to all buttons
        :param ptr: Widget pointer
        :return: nothing
        """
        if self.view.play_button.playing:
            self.pub.set_game_state(0)  # Sets the game state to stopped
            self.pub.publish()

            for button in self.view.action_buttons:
                button.activate()
            ptr.color(fl.FL_DARK_GREEN)
            ptr.label("Jogar")
            self.view.play_button.playing = False
        else:
            if ptr.id == 4:
                self.pub.set_game_state(1)  # Sets the game state to normal play
                print("Jogar regular")
            elif ptr.id == 0:
                self.pub.set_game_state(2)
                print("Free Ball")
            elif ptr.id == 1:
                self.pub.set_game_state(3)
                print("Penalty")
            elif ptr.id == 2:
                self.pub.set_game_state(4)
                print("Meta")
            else:
                print("que")

            self.pub.publish()

            for button in self.view.action_buttons:
                button.deactivate()

            self.view.play_button.color(fl.FL_RED)
            self.view.play_button.label("Parar")
            self.view.play_button.playing = not self.view.play_button.playing

    def on_color_change(self, ptr):
        """
        This function change the team color in the game_opt to the selected one
        :param ptr: pointer of the widget
        :return: nothing
        """
        value = ptr.value()
        self.pub.set_team_color(value)
        self.pub.publish()
        self.game_opt["time"] = value
        self.set_robot_plot_color(value)

    def set_robot_plot_color(self, is_yellow=True):
        if is_yellow:
            self.view.home_color = 1  # home team color
            self.view.away_color = 0  # away team color
        else:
            self.view.home_color = 0  # home team color
            self.view.away_color = 1  # away team color

    def on_side_change(self, ptr):
        """
        This function changes the team side in game_opt to the selected one
        :param ptr: pointer of the widget
        :return: nothing
        """
        self.pub.set_team_side(ptr.value())
        self.game_opt["side"] = ptr.value()
        self.pub.publish()
        self.view.virtualField.attack_goal = not ptr.value()
