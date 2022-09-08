# -*- coding: utf-8 -*-
from interface.View.ConnectionView import ConnectionView
from fltk import fl_message, FL_WHITE, Fl
from utils.enum_interfaces import all_interfaces
from os import environ


class ConnectionController:
    """
        This class is responsible for creating and updating the connection view

        :return: returns nothing
    """

    def __init__(self, _game_opt):
        # Creating the view for displaying information
        self.view = ConnectionView()
        # Default rosmaster
        self.default_ros_uri = "http://localhost:11311"
        # stores the game options for future use
        self.game_opt = _game_opt
        # Reading the ROS_MASTER_URI from file
        self.file_master_uri = self.game_opt["ROS_MASTER_URI"]
        self.view.ip_field.value(self.file_master_uri)  # Sets the value for input

        # Setting callbacks
        self.view.update_button.callback(self.change_ros_master)
        self.view.self_ip_field.callback(self.ip_choice)

        # the -1 item is set when a foreign interface is set
        # usually this happens when multiple computers share the same data-base
        item = -1
        count = 0  # aux value for iterating over interfaces

        self.ip_list = all_interfaces()  # list all interfaces and their respective ids

        for ip in self.ip_list:
            # add item for the choice menu
            self.view.self_ip_field.add(ip[0]+" - "+ ip[1])
            # If the ip and interface is correct, this is in fact, the correct network
            if ip[0] == self.game_opt['ROS_IP'][0] and ip[1] == self.game_opt['ROS_IP'][1]:
                # So we store its position for setting the choice menu later
                item = count
            count += 1

        # If no valid interfaces are found, we append this to end of choice menu
        # TODO: red color for foreign interfaces
        if item == -1:
            item = count # count is the last item
            self.view.self_ip_field.add(self.game_opt['ROS_IP'][0] + " - " + self.game_opt['ROS_IP'][1])
            self.ip_list.append(self.game_opt['ROS_IP'])  # add item for the choice menu

        # set the value choice
        self.view.self_ip_field.value(item)
        # and the current ip for saving the variable
        self.ip = self.ip_list[item]

    def show(self):
        """
            Displays the connection view
            :return: returns nothing
        """
        self.view.root.show()

    def ip_choice(self, ptr):
        """
            This function sets the ip variable for saving
            :param ptr: pointer for the widget, uses value() attribute
            :return: returns nothing
        """
        self.ip = self.ip_list[ptr.value()]

    def change_ros_master(self, ptr):
        """
            This function is for updating the bash and json

            :param ptr: not used but required for callback
            :return: returns nothing
        """

        self.file_master_uri = self.view.ip_field.value()  # getting the value from input
        formatted_ip_string = self.ip[1]  # turns a 4 characters string in to readable ip
        self.game_opt["ROS_MASTER_URI"] = self.file_master_uri
        self.game_opt["ROS_IP"] = [self.ip[0], formatted_ip_string]

        # the entire file cant be stored in one variable
        line = "export ROS_MASTER_URI="+self.file_master_uri + "\n" +\
               "export ROS_HOSTNAME="+formatted_ip_string + "\n" +\
               "export ROS_IP="+formatted_ip_string + "\n"

        with open(environ['ROS_ARARA_ROOT'] + "src/parameters/rosmaster.bash", "w+") as bash_file:
            # writes lines to file
            bash_file.write(line)
            bash_file.close()
            # background is different for alert messages
            Fl.background(200, 200, 200)
            fl_message("Por favor, reinicie o bash para concluir as alterações")
