# -*- coding: utf-8 -*-
from interface.View.RobotParamsView import RobotParamsView


class RobotParamsController:
    """This class controls the subwindow of Robot Parameters"""
    def __init__(self, _robot_params, _robot_bluetooth, _robot_roles, _robot_bodies):
        """
        :param _robot_params: Dict
        :param _robot_bluetooth: Dict
        :param _robot_roles: Dict
        :param _robot_bodies: Dict
        :return: nothing
        """
        # Save the parameters for future use
        self.robot_params = _robot_params
        self.robot_bluetooth = _robot_bluetooth
        self.robot_roles = _robot_roles
        self.robot_bodies = _robot_bodies

        # Init variables
        self.current_robot = None
        self.bluetooth_value = None
        self.body_value = None
        self.bluetooth_is_owned_by = None
        self.body_is_owned_by = None
        self.tag_is_owned_by = None

        # View
        self.view = RobotParamsView()

        # Fast access array to use a dict as an simple array
        self.faster_hash = ['robot_' + str(x) for x in range(1, 6)]
        self.fast_text = ['Jogador ' + str(x) for x in range(1, 6)]

        # Callback handling
        self.view.cancel_button.callback(self.hide)
        self.view.ok_button.callback(self.save_and_exit)

        self.view.end(True)

    def set_owner_of_parameters(self, n_robots=5):
        """
        Defines which options are available
        :param n_robots: Number of robots to check ownership
        :return: nothing
        """
        # Give the correct owner for things
        for x in range(n_robots):
            current_robot = self.robot_params[self.faster_hash[x]]

            try:  # assort tag
                current_robot_tag = current_robot['tag_number']
                if self.tag_is_owned_by[int(current_robot_tag)] is None:  # in case tag is empty
                    # The current robot is the owner of that tag
                    self.tag_is_owned_by[int(current_robot_tag)] = self.faster_hash[x]

            except KeyError:
                pass

            try:  # assort the robot bluetooth
                current_robot_bluetooth = current_robot['bluetooth_mac_address']
                if self.bluetooth_is_owned_by[current_robot_bluetooth] is None:
                    self.bluetooth_is_owned_by[current_robot_bluetooth] = self.faster_hash[x]
            except KeyError:
                pass

            try:
                current_robot_body = current_robot['body_id']
                if self.body_is_owned_by[current_robot_body] is None:
                    self.body_is_owned_by[current_robot_body] = self.faster_hash[x]
            except KeyError:
                pass

    def clear_variables(self):
        """
        Clear choices and set value to none
        :return: nothing
        """
        # Saves current robot owner of bluetooth
        self.bluetooth_is_owned_by = dict.fromkeys(self.robot_bluetooth.keys())

        # Saves current robot owner of body
        self.body_is_owned_by = dict.fromkeys(self.robot_bodies.keys())

        # Do the same for tag
        self.tag_is_owned_by = [None]*20

        self.view.tag_field.clear()
        self.view.body_field.clear()
        self.view.bluetooth_field.clear()

    def populate_fields(self):
        """
        Inflate choice inputs with dict values
        :return: nothing
        """
        # Values for mapping int to bluetooth name
        self.bluetooth_value = {"nenhum": 0}
        self.view.bluetooth_field.add("nenhum")

        value = 1  # Value for bluetooth choices
        for key in self.bluetooth_is_owned_by:
            self.bluetooth_value[key] = value
            if self.bluetooth_is_owned_by[key] is not None:
                key += '*'
            self.view.bluetooth_field.add(key)
            value += 1

        # Values for mapping, in the same way, the body name
        self.body_value = {"nenhum": 0}
        self.view.body_field.add("nenhum")

        value = 1  # Value for body choices
        for key in self.body_is_owned_by:
            self.body_value[key] = value
            if self.body_is_owned_by[key] is not None:
                key += '*'
            self.view.body_field.add(key)
            value += 1

        # Add tags to tag input
        for x in range(20):
            if self.tag_is_owned_by[x] is not None:
                self.view.tag_field.add(str(x)+'*')
            else:
                self.view.tag_field.add(str(x))

    def hide(self, ptr):
        """
        Just hide the window
        :param ptr: Button Pointer
        :return: nothing
        """
        self.view.root.hide()

    def show(self, robot_id):
        """
        Clear inputs, define owners, define robot_id
        :param robot_id: int
        :return: nothing
        """
        self.clear_variables()
        self.set_owner_of_parameters()
        self.populate_fields()
        self.set_robot_params(robot_id)
        self.view.root.show()

    def save_and_exit(self, ptr):
        """
        Defines dict values for robot_id
        :param ptr: button pointer
        :return: nothing
        """
        self.current_robot["tag_number"] = int(self.view.tag_field.text().strip('*'))
        self.current_robot["bluetooth_mac_address"] = self.view.bluetooth_field.text().strip('*')
        self.current_robot["body_id"] = self.view.body_field.text().strip('*')
        self.hide(None)

    def set_robot_params(self, robot_id):
        """
        This class sets the window parameters for the the given robot id
        :param robot_id: int
        :return: nothing
        """
        self.current_robot = current_robot = self.robot_params[self.faster_hash[robot_id]]

        self.view.title.label(self.fast_text[robot_id])

        try:
            self.view.tag_field.value(
                int(current_robot['tag_number'])
            )
        except KeyError:
            self.view.tag_field.value(0)

        try:
            self.view.bluetooth_field.value(
                self.bluetooth_value[current_robot['bluetooth_mac_address']]
            )
        except KeyError:
            self.view.bluetooth_field.value(0)

        try:
            self.view.body_field.value(
                self.body_value[current_robot['body_id']]
            )
        except KeyError:
            self.view.body_field.value(0)


if __name__ == '__main__':
    from interface.View.RobotParamsView import RobotParamsView
    from utils.model import Model
    model = Model()
    r = RobotParamsController(model.robot_params, model.robot_bluetooth, model.robot_roles, model.robot_bodies)
    r.show(0)
    r.view.end(False)