# -*- coding: utf-8 -*-

from interface.View.CameraSelectView import CameraSelectView
import fltk as fl
import sys
from os import path

class CameraSelectController:

    def update_list(self, camera_list):
        self.view.device_input.clear()
        self.view.device_input.add("Nenhum")
        self.view.device_input.value(0)
        self.view.device_input.add("Leitura de arquivo")
        self.choice_map = {}

        for camera in camera_list:
            index = self.view.device_input.add(
                camera + " - " + camera_list[camera]["name"] + " (" + camera_list[camera]["product"] + ")")
            self.choice_map[index] = camera

        self.view.device_input.redraw()

    def window_system_exit(self,ptr):
        sys.exit(0)

    def button_callback(self,ptr):
        if ptr.id == 0:  # Cancel
            sys.exit(0)
        elif ptr.id == 1:  # Ok Button
            self.view.root.hide()
        else:  # Refresh
            self.callable()
            self.update_list(self.camera_list)

    def browser_callback(self, ptr):
        fl.Fl.background(200, 200, 200)
        self.view.file_browser.show()

        while self.view.file_browser.visible():
            fl.Fl.wait()

        self.view.file_box.align(fl.FL_ALIGN_INSIDE|fl.FL_ALIGN_RIGHT)
        self.view.file_box.label(self.view.file_browser.value())
        self.device = self.view.file_browser.value()
        self.camera_model['file'] = self.device

    def input_callback(self, ptr):

        item = ptr.value() - 2
        if item == -1:
            self.return_type = 1
            self.view.file_browser_label.show()
            self.view.file_button.show()
            self.view.file_box.show()
        else:
            if item == -2:
                self.return_type = 0
                #self.device = None
            else:
                self.return_type = 2
                self.device = self.choice_map[ptr.value()]

            self.view.file_browser_label.hide()
            self.view.file_button.hide()
            self.view.file_box.hide()

    def __init__(self, _camera_model, _camera_list, _callable):
        self.camera_model = _camera_model
        self.camera_list = _camera_list
        self.callable = _callable
        self.view = CameraSelectView()

        self.return_type = 0
        self.device = None
        self.choice_map = None

        self.view.warning.label("Dispositivo \""+self.camera_model["name"]+"\" n�o encontrado.")

        self.update_list(self.camera_list)

        if path.isfile(self.camera_model['file']):
            self.view.device_input.value(1)
            self.return_type = 1
            self.view.file_box.align(fl.FL_ALIGN_INSIDE | fl.FL_ALIGN_RIGHT | fl.FL_ALIGN_WRAP)
            self.view.file_box.label(self.camera_model['file'])
            self.device = self.camera_model['file']
        else:
            self.view.file_browser_label.hide()
            self.view.file_button.hide()
            self.view.file_box.hide()

        self.view.device_input.callback(self.input_callback)
        self.view.file_button.callback(self.browser_callback)

        self.view.cancel_button.id = 0
        self.view.ok_button.id = 1
        self.view.refresh_button.id = 2

        self.view.root.callback(self.window_system_exit)
        self.view.cancel_button.callback(self.button_callback)
        self.view.ok_button.callback(self.button_callback)
        self.view.refresh_button.callback(self.button_callback)

        self.view.end()
