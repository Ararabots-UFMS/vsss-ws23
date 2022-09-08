# -*- coding: utf-8 -*-

import fltk as fl
from .ImageCreator import ImageCreator
from os import path, environ
import time
from threading import Thread


class LoadingView(Thread):

    def __init__(self, label_text = "Carregando Assets" ):
        Thread.__init__(self)
        # Feio
        self.DoRun = True

        # Get the usable screen proportions
        self.width = fl.Fl.w()
        self.height = fl.Fl.h()

        self.root = fl.Fl_Single_Window(self.proportion_width(32), self.proportion_height(40),
                                        self.proportion_width(46), self.proportion_height(20))
        self.width = self.root.w()
        self.height = self.root.h()

        self.root.border(0)
        fl.Fl.background(23, 23, 23)
        self.root.labelcolor(fl.FL_WHITE)

        logo_width = self.proportion_width(28)
        logo_heigth = self.proportion_height(90)
        self.logo = fl.Fl_Box(self.proportion_width(1),self.proportion_height(5), logo_width, logo_heigth)

        root_path = environ['ROS_ARARA_ROOT']

        logo_file = root_path+"src/interface/Assets/Cache/arara_"+str(logo_width)+"_"+str(logo_heigth)+".png"
        if path.exists(logo_file):
            self.logo_image = fl.Fl_PNG_Image(logo_file)
            self.logo.image(self.logo_image)
        else:
            ImageCreator(logo_width, logo_heigth)
            if path.exists(logo_file):
                self.logo_image = fl.Fl_PNG_Image(logo_file)
                self.logo.image(self.logo_image)
            else:
                print(logo_file)
                print("nope")
                self.logo_image = None

        self.label = fl.Fl_Box(self.proportion_width(31),self.proportion_height(5),self.proportion_width(68),
                               self.proportion_height(90),label_text)

        self.label.box(fl.FL_FLAT_BOX)
        self.label.align(fl.FL_ALIGN_INSIDE + fl.FL_ALIGN_LEFT +fl.FL_ALIGN_WRAP)
        self.label.labelcolor(fl.FL_WHITE)  # color
        self.label.labelsize(22)  # Font Size
        self.label.labelfont(fl.FL_HELVETICA_BOLD)  # Bold type

        self.label_position = 0
        self.labels_text = ["Salvando banco de dados",
                            "Carregando n� da vis�o",
                            "Testando texto desnecess�rio e extremamente longo para essa tela",
                            "Carregando Assets",
                            "Carregando..."]

        self.root.end()
        self.root.show()

    def proportion_height(self, proportion):
        """Returns the Y value for the designed vertical screen proportion"""
        return int(self.height * proportion / 100)

    def proportion_width(self, proportion):
        """Returns the X value for the designed horizontal screen proportion"""
        return int(self.width * proportion / 100)

    def set_and_show(self, label_text):
        self.label.label(label_text)
        #if not self.root.visible():
        self.root.show()

    def close(self):
        self.root.hide()

    def set_label(self):
        self.label.label(self.labels_text[self.label_position])
        self.label_position =(self.label_position+1)%5
        fl.Fl.add_timeout(c.RATE, c.set_label)

    def run(self):
        while self.DoRun:
            fl.Fl.check()
            time.sleep(0.1)
        self.root.hide()


if __name__ == '__main__':
    c = LoadingView()
    #c.end()

    c.RATE = 1.5

    fl.Fl.add_timeout(c.RATE, c.set_label)

    while(1):
        fl.Fl.check()
    #fl.Fl.run()