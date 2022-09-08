# -*- coding: utf-8 -*-

import fltk as fl
import ctypes as ctype
import sys
import os

from time import sleep


def theCancelButtonCallback(ptr):
    os._exit(-1)


class WindowManager:
    """docstring for window_manager"""

    def __init__(self):
        self.width = fl.Fl.w()
        self.height = fl.Fl.h()
        self.root = fl.Fl_Double_Window(self.proportion_width(2.5), self.proportion_height(5),
                                 self.proportion_width(45), self.proportion_height(55))
        self.root.label("Conexao")
        self.create_main_title("DEBUG")
        self.check_simulacao = None
        self.title_visao = None
        self.create_title_visao("Visão")
        self.scroll = fl.Fl_Scrollbar(self.proportion_width(0), self.proportion_height(0), self.proportion_width(100), self.proportion_height(100))
        self.scroll.align(fl.FL_ALIGN_RIGHT+fl.FL_ALIGN_TOP)
        # self.scroll.end()

        self.title_console = None
        self.create_title_console("Console")
        self.check_robots = None
        self.create_check_robots()

        self.create_check_simulacao()
        self.create_check_vetores()
        self.top_menu = None
        self.line = None
        self.title = None
        self.root.resizable(self.scroll)
        self.root.end()
        fl.Fl.background(23, 23, 23)
        self.root.labelcolor(fl.FL_WHITE)
        self.root.show(len(sys.argv), sys.argv)
        fl.Fl.run()
        while fl.Fl.wait() > 0:
            if fl.Fl.get_key(113):
                fl.Fl.exit()

    def create_main_title(self, text):
        self.title = fl.Fl_Box(self.proportion_width(5), self.proportion_height(0),
                               self.proportion_width(35), self.proportion_height(10), text)
        self.title.labelcolor(fl.FL_WHITE)
        self.title.labelsize(23)
        self.title.box(fl.FL_FLAT_BOX)
        # self.title.color(fl.FL_RED)
        self.title.align(fl.FL_ALIGN_CENTER)
        self.title.show()

    def create_title_visao(self, text):
        self.title = fl.Fl_Box(self.proportion_width(5), self.proportion_height(10),
                               self.proportion_width(35), self.proportion_height(4), text)
        self.title.labelcolor(fl.FL_WHITE)
        self.title.labelsize(23)
        self.title.box(fl.FL_NO_BOX)
        # self.title.color(fl.FL_RED)
        self.title.align(fl.FL_ALIGN_CENTER)
        self.title.show()

    def create_check_simulacao(self):
        self.check_simulacao = None
        self.check_simulacao = fl.Fl_Check_Button(self.proportion_width(11),
                           self.proportion_height(15),
                           self.proportion_width(4), self.proportion_height(4), "Simulação de previsão de movimento"
                           )
        self.check_simulacao.labelcolor(fl.FL_WHITE)
        self.check_simulacao.clear_visible_focus()

    def create_check_vetores(self):
        self.check_vetores = None
        self.check_vetores = fl.Fl_Check_Button(self.proportion_width(17),
                           self.proportion_height(20),
                           self.proportion_width(4), self.proportion_height(4), "Vetores dos robôs"
                           )
        self.check_vetores.labelcolor(fl.FL_WHITE)
        self.check_vetores.clear_visible_focus()

    def create_title_console(self, text):
        self.title_console = fl.Fl_Box(self.proportion_width(5), self.proportion_height(26),
                               self.proportion_width(35), self.proportion_height(4), text)
        self.title_console.labelcolor(fl.FL_WHITE)
        self.title_console.labelsize(23)
        self.title_console.box(fl.FL_FLAT_BOX)
        # self.title.color(fl.FL_RED)
        self.title_console.align(fl.FL_ALIGN_CENTER)
        self.title_console.show()

    def create_check_robots(self):
        n = 8
        self.check_robots = [None]*n
        temp_names = ["robo 1", "robo 2", "robo 3", "robo 4","robo 5", "robo 6", "robo 7", "robo 8", ]
        print(temp_names)
        for num in range(n):
            self.check_robots[num] = fl.Fl_Check_Button(self.proportion_width(2)*n + self.proportion_width(5),
                               self.proportion_height(30) + self.proportion_height(3) * num * 2,
                               self.proportion_width(4), self.proportion_height(4),temp_names[num]
                               )
            self.check_robots[num].labelcolor(fl.FL_WHITE)
            self.check_robots[num].clear_visible_focus()

    def create_scroll(self):
        self.scroll = fl.Fl_Scrollbar(self.proportion_width(5), self.proportion_height(26),
                               self.proportion_width(35), self.proportion_height(20))


    def proportion_height(self, proportion):
        return int(self.height * proportion/100)

    def proportion_width(self, proportion):
        return int(self.width * proportion/100)


if __name__ == '__main__':

    window_manager = WindowManager()

    fl.Fl.run()
