# -*- coding: utf-8 -*-

from fltk import Fl_Single_Window, Fl, Fl_Box, FL_WHITE, Fl_Choice, FL_HELVETICA_BOLD, FL_RED, Fl_File_Chooser, \
    FL_FLAT_BOX, FL_DARK_GREEN, Fl_Button, FL_BLUE


class CameraSelectView:

    def __init__(self):

        # Get the usable screen proportions
        self.width = Fl.w()
        self.height = Fl.h()

        self.root = Fl_Single_Window(self.proportion_width(30), self.proportion_height(25),
                                        self.proportion_width(40), self.proportion_height(50),
                                     "Selecionar Dispositivo de Captura")
        self.width = self.root.w()
        self.height = self.root.h()

        self.padding_y = self.proportion_height(10)

        self.warning = Fl_Box(self.proportion_width(15),
                              self.padding_y,
                              self.proportion_width(68),
                              self.proportion_height(10), "Dispositivo '#DISP_ANTIGO' n�o encontrado.")

        self.padding_y += self.proportion_height(20)

        self.device_input = Fl_Choice(self.proportion_width(40),
                                      self.padding_y,
                                      self.proportion_width(40),
                                         self.proportion_height(7),
                                         "Novo dispositivo:")

        self.padding_y += self.proportion_height(20)

        self.file_browser = Fl_File_Chooser(".", "*", Fl_File_Chooser.SINGLE, "Selecione o arquivo de v�deo")

        self.file_browser_label = Fl_Box(self.proportion_width(16),
                                          self.padding_y,
                                          self.proportion_width(25),
                                          self.proportion_height(5), "Arquivo de v�deo:")

        self.padding_y += self.proportion_height(7)

        self.file_box = Fl_Box(self.proportion_width(15),
                                          self.padding_y,
                                          self.proportion_width(58),
                                          self.proportion_height(7), "")

        self.file_button = Fl_Button(self.proportion_width(73),
                                          self.padding_y,
                                          self.proportion_width(7),
                                          self.proportion_height(7), "@+")

        self.padding_y += self.proportion_height(30)

        self.cancel_button = Fl_Button(self.proportion_width(8),
                                     self.padding_y,
                                     self.proportion_width(25),
                                     self.proportion_height(10), "Cancel")

        self.refresh_button = Fl_Button(self.proportion_width(38),
                                     self.padding_y,
                                     self.proportion_width(25),
                                     self.proportion_height(10), "Atualizar")

        self.ok_button = Fl_Button(self.proportion_width(68),
                                     self.padding_y,
                                     self.proportion_width(25),
                                     self.proportion_height(10), "OK")

        # Define colors

        self.apply_bold_and_color(self.ok_button, FL_DARK_GREEN, True)
        self.apply_bold_and_color(self.cancel_button, FL_RED, True)
        self.apply_bold_and_color(self.refresh_button, FL_BLUE, True)

        self.file_button.box(FL_FLAT_BOX)
        self.file_button.labelcolor(FL_WHITE)
        self.file_button.color(FL_DARK_GREEN)

        self.file_box.box(FL_FLAT_BOX)
        self.file_box.color(FL_WHITE)

        self.apply_bold_and_color(self.file_browser_label)

        self.apply_bold_and_color(self.warning)

        self.device_input.color(FL_RED)
        self.apply_bold_and_color(self.device_input)

        Fl.background(23, 23, 23)

    def apply_bold_and_color(self, label, color=0, flat_box = False):
        label.labelcolor(FL_WHITE)
        label.labelfont(FL_HELVETICA_BOLD)
        if color:
            label.color(color)

        if flat_box:
            label.box(FL_FLAT_BOX)

    def proportion_height(self, proportion):
        """Returns the Y value for the designed vertical screen proportion"""
        return int(self.height * proportion / 100)

    def proportion_width(self, proportion):
        """Returns the X value for the designed horizontal screen proportion"""
        return int(self.width * proportion / 100)

    def end(self):

        self.root.end()
        self.root.show()

        Fl.run()