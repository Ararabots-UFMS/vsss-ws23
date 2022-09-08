# -*- coding: utf-8 -*-

import fltk as fl
import ctypes as ctype
import sys
import os

from time import sleep


def theCancelButtonCallback(ptr):
    os._exit(-1)


class ConnectionView:
    """View responsible for creating the Connection Window"""

    def __init__(self):
        # get the current desktop size
        self.width = fl.Fl.w()
        self.height = fl.Fl.h()

        # using current desktop sizes, create a connection window
        self.root = fl.Fl_Double_Window(self.proportion_width(2.5), self.proportion_height(5),
                                 self.proportion_width(45), self.proportion_height(55))

        # update the current dimensions for the ones given by the window
        self.width = self.root.w()
        self.height = self.root.h()

        # Allocating resources
        self.top_menu = None
        self.line = None
        self.title = None
        self.node_title = None
        self.bluetooth_title = None
        self.carcaca_title = None
        self.self_ip_field = None
        self.choice_jogador = []
        self.check_robots = None
        self.ip_field = None
        self.update_button = None
        self.joystick_title = None
        # end of resource allocation

        self.root.label("Conexao")  # window title
        self.create_main_title("Node mestre")  # main window title
        self.create_ip_field()  # inputs
        self.create_update_button("Atualizar")  # last button
        fl.Fl.background(23, 23, 23)
        self.root.labelcolor(fl.FL_WHITE)  # font color for current window
        self.root.end()  # end and show

    def create_main_title(self, text):
        """
            This function creates the main windows title
            :param: text: String
            :return: returns nothing
        """

        self.title = fl.Fl_Box(self.proportion_width(33), self.proportion_height(5),
                               self.proportion_width(34), self.proportion_height(10), text)
        self.title.labelcolor(fl.FL_WHITE)
        self.title.labelsize(23)
        self.title.box(fl.FL_FLAT_BOX)

        self.title.align(fl.FL_ALIGN_CENTER)
        self.title.show()

    def create_ip_field(self):
        """
            This function creates both inputs for ip

            :return: returns nothing
        """

        self.ip_field = fl.Fl_Input(self.proportion_width(30), self.proportion_height(25),
                               self.proportion_width(40), self.proportion_height(8), "IP Nó Mestre:")
        self.ip_field.align(fl.FL_ALIGN_LEFT_TOP)
        self.ip_field.labelcolor(fl.FL_WHITE)
        self.ip_field.show()

        self.self_ip_field = fl.Fl_Choice(self.proportion_width(30), self.proportion_height(45),
                               self.proportion_width(40), self.proportion_height(8), "Próprio ROS IP:")
        self.self_ip_field.align(fl.FL_ALIGN_LEFT_TOP)
        self.self_ip_field.labelcolor(fl.FL_WHITE)
        self.self_ip_field.show()

    def create_update_button(self,text):
        """
            This function creates the update button
            :param: text: String
            :return: returns nothing
        """

        self.update_button = fl.Fl_Button(self.proportion_width(38), self.proportion_height(65),
                               self.proportion_width(24), self.proportion_height(6), text)
        self.update_button.color(fl.FL_RED)
        self.update_button.labelcolor(fl.FL_WHITE)
        self.update_button.labelfont(fl.FL_BOLD)

    def proportion_height(self, proportion):
        return int(self.height * proportion/100)

    def proportion_width(self, proportion):
        return int(self.width * proportion/100)


if __name__ == '__main__':

    window_manager = ConnectionView()
    window_manager.root.show()
    fl.Fl.run()