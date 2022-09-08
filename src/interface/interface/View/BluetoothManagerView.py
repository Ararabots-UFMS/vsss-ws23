# -*- coding: utf-8 -*-

import fltk as fl
import ctypes as ctype
import sys
import os

from time import sleep

def theCancelButtonCallback(ptr):
    os._exit(-1)

class BluetoothManagerView:
    """docstring for window_manager"""

    def __init__(self):
        self.width = fl.Fl.w()
        self.height = fl.Fl.h()
        self.root = fl.Fl_Double_Window(self.proportion_width(2.5), self.proportion_height(5),
                                 self.proportion_width(45), self.proportion_height(55))
        self.root.label("Bluetooth manager")
        self.title = None
        self.new_name = None #used to get name from add window
        self.new_address = None #used to get address from add window
        self.create_main_title("Bluetooth")

        self.scroll = fl.Fl_Scroll(0,0, self.root.w()-2, self.root.h())
        self.scroll.box(fl.FL_BORDER_BOX)
        scrollbar = self.scroll.getScrollbar()
        self.scroll.add(self.title)
        self.scroll.end()

        self.bluetooths = []
        self.create_add_button()

        self.root.resizable(self.scroll)    #allow to create scrollbar
        self.root.show()
        fl.Fl.background(23, 23, 23)
        self.root.labelcolor(fl.FL_WHITE)
        # self.root.show(len(sys.argv), sys.argv)
        # for i in range(10):
        #     self.create_bluetooth_entry("name","address")
        # self.root.end()
        # while fl.Fl.wait() > 0:
        #     # print(self.bluetooths[-1][0].label())
        #     if fl.Fl.get_key(ord("q")):#ord pega o numero do caractere dentro da funcd
        #         fl._exit()#sair da janela
        #     elif fl.Fl.get_key(ord("n")):
        #         self.add_button_cb(None)

    #this method creates the main title of the window
    def create_main_title(self, text):
        self.title = fl.Fl_Box(self.proportion_width(5), self.proportion_height(0),
                               self.proportion_width(35), self.proportion_height(10), text)
        self.title.labelcolor(fl.FL_WHITE)
        self.title.labelsize(23)
        self.title.box(fl.FL_FLAT_BOX)
        self.title.align(fl.FL_ALIGN_CENTER)
        self.title.show()

    def create_add_button(self):
        self.add_button = fl.Fl_Button(self.proportion_width(30), self.proportion_height(3),
                               self.proportion_width(5), self.proportion_height(4), "Novo")
        self.add_button.callback(self.add_button_cb)
        self.add_button.labelcolor(fl.FL_WHITE)
        self.add_button.color(fl.FL_DARK_GREEN)
        self.add_button.clear_visible_focus()
        self.add_button.labelfont(fl.FL_BOLD)
        self.add_button.labelcolor(fl.FL_WHITE)
        self.add_button.show()
        self.scroll.add(self.add_button)

    def add_button_cb(self,ptr):
        self.create_add_window()
        if self.new_name != None and self.new_address != None:
            self.create_bluetooth_entry(self.new_name, self.new_address)
        self.new_name = None
        self.new_address = None

    def create_add_window(self, name = None, address = None):

        self.add_window = fl.Fl_Single_Window(420, 300, "Adicionar Bluetooth")
        self.add_window.callback(self.add_window_cb)
        self.add_name_label = fl.Fl_Box(0, self.proportion_height(5), self.proportion_width(6), self.proportion_height(5), "Nome")
        self.add_name_label.labelcolor(fl.FL_WHITE)
        self.add_name_input = fl.Fl_Input(self.proportion_width(8), self.proportion_height(5), self.proportion_width(15), self.proportion_height(5))

        self.add_address_label = fl.Fl_Box(0, self.proportion_height(14), self.proportion_width(6), self.proportion_height(5), "Endereço")
        self.add_address_label.labelcolor(fl.FL_WHITE)
        self.add_address_input = fl.Fl_Input(self.proportion_width(8), self.proportion_height(14), self.proportion_width(15), self.proportion_height(5))

        self.add_new_button = fl.Fl_Button(self.proportion_width(11), self.proportion_height(22),self.proportion_width(6), self.proportion_height(5), "Salvar" )
        self.add_new_button.color(fl.FL_DARK_GREEN)
        self.add_new_button.labelcolor(fl.FL_WHITE)
        self.add_new_button.labelfont(fl.FL_BOLD)
        self.add_new_button.callback(self.add_new_button_cb)
        if name != None or address != None:
            self.add_name_input.value(str(name))
            self.add_address_input.value(str(address))

        self.add_button.deactivate()
        self.add_window.show()

        while self.add_window.visible():
            fl.Fl.wait()
        self.add_button.activate()

    def add_window_cb(self, ptr):
        ptr.hide()
        pass

    def add_new_button_cb(self,ptr):
        self.new_name = self.add_name_input.value()
        self.new_address = self.add_address_input.value()
        self.add_button.activate()
        self.add_window.hide()


    def create_bluetooth_entry(self, name, address):
        if len(self.bluetooths) == 0:
            current_height = 0
        else:
            current_height = self.bluetooths[-1][0].y()
        bluetooth_name = fl.Fl_Box(self.proportion_width(4), self.proportion_height(7) + current_height,
                                self.proportion_width(10), self.proportion_height(10), name)
        bluetooth_name.labelcolor(fl.FL_WHITE)
        bluetooth_name.labelsize(14)
        bluetooth_name.show()

        bluetooth_address = fl.Fl_Box(self.proportion_width(16), self.proportion_height(7) + current_height,
                                self.proportion_width(10), self.proportion_height(10), address)
        bluetooth_address.labelcolor(fl.FL_WHITE)
        bluetooth_address.color(fl.FL_RED)
        bluetooth_address.labelsize(14)
        bluetooth_address.show()

        bluetooth_edit_button = fl.Fl_Button(self.proportion_width(30), self.proportion_height(10) + current_height,
                                self.proportion_width(5), self.proportion_height(4), "Editar")
        bluetooth_edit_button.color(fl.FL_RED)
        bluetooth_edit_button.clear_visible_focus()
        bluetooth_edit_button.labelfont(fl.FL_BOLD)
        bluetooth_edit_button.labelcolor(fl.FL_WHITE)
        bluetooth_edit_button.callback(self.bluetooth_edit_button_cb, (bluetooth_name.label(), bluetooth_address.label(), len(self.bluetooths)))
        # bluetooth_edit_button.show()

        bluetooth_delete_button = fl.Fl_Button(self.proportion_width(36), self.proportion_height(10) + current_height,
                                self.proportion_width(5), self.proportion_height(4), "Excluir")
        bluetooth_delete_button.color(fl.FL_RED)
        bluetooth_delete_button.clear_visible_focus()
        bluetooth_delete_button.labelfont(fl.FL_BOLD)
        bluetooth_delete_button.labelcolor(fl.FL_WHITE)
        bluetooth_delete_button.callback(self.bluetooth_delete_button_cb, len(self.bluetooths))
        # bluetooth_delete_button.show()

        self.bluetooths.append([bluetooth_name, bluetooth_address, bluetooth_edit_button, bluetooth_delete_button])
        for i in self.bluetooths[-1]:
            self.scroll.add(i)
        self.scroll.redraw()

    def bluetooth_edit_button_cb(self, ptr, tupla):
        name, address, pos = tupla
        self.create_add_window(name, address)
        if self.new_name != None or self.new_address != None:
            self.bluetooths[pos][0].label(self.new_name)
            self.bluetooths[pos][1].label(self.new_address)
        self.new_name = None
        self.new_address = None
        self.scroll.redraw()

    def bluetooth_delete_button_cb(self, ptr, pos):
        self.scroll.remove(self.bluetooths[pos][0])
        self.scroll.remove(self.bluetooths[pos][1])
        self.scroll.remove(self.bluetooths[pos][2])
        self.scroll.remove(self.bluetooths[pos][3])
        # fl.Fl_delete_widget(self.bluetooths[pos][0])
        # fl.Fl_delete_widget(self.bluetooths[pos][1])
        # self.bluetooths[pos][0].
        fl.Fl_delete_widget(self.bluetooths[pos][2])
        fl.Fl_delete_widget(self.bluetooths[pos][3])
        for i in range(pos,len(self.bluetooths)-1):
            self.bluetooths[i] = self.bluetooths[i+1]
            self.bluetooths[i][3].callback(self.bluetooth_delete_button_cb, i)
            # self.bluetooths[i][1] = self.bluetooths[i][1]
            # self.bluetooths[i][2].callback(self.bluetooth_edit_button_cb, (self.bluetooths[i][0].label(), self.bluetooths[i][1].label(), i))
            # self.bluetooths[i][3].callback(self.bluetooth_delete_button_cb, i)
        self.bluetooths.pop()
        self.new_bluetooth_position()
        self.scroll.redraw()

    def new_bluetooth_position(self):
        current_height = 0
        for i in range(len(self.bluetooths)):
            y  = self.proportion_height(7) + current_height
            self.bluetooths[i][0].position(self.bluetooths[i][0].x(),y)
            self.bluetooths[i][1].position(self.bluetooths[i][1].x(),y)
            y  = self.proportion_height(10) + current_height
            self.bluetooths[i][2].position(self.bluetooths[i][2].x(),y)
            self.bluetooths[i][3].position(self.bluetooths[i][3].x(),y)
            current_height = y
        # fl.Fl_delete_widget(self.scroll)
        self.scroll.redraw()

    def end(self, hidden=False):
        # Show main window
        self.root.clear_visible_focus()
        self.root.end()
        if hidden:
            self.root.hide()
        else:
            self.root.show()
            fl.Fl.run()


    def window_callback(self,ptr):
        self.view.root.hide()
        return self.bluetooths

    def proportion_height(self, proportion):
        return int(self.height * proportion/100)

    def proportion_width(self, proportion):
        return int(self.width * proportion/100)

if __name__ == '__main__':
    window_manager = BluetoothManagerView()
    window_manager.end()
