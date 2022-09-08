# -*- coding: utf-8 -*-

from interface.View.BluetoothManagerView import BluetoothManagerView
import fltk as fl
from utils.json_handler import JsonHandler

class BluetoothManagerController():
    def __init__(self, _model, hidden=False):
        self.model = _model#self.json_handler.read("parameters/bluetooth.json")
        self.bluetooths_dict = self.model.robot_bluetooth
        self.view = BluetoothManagerView()
        self.view.root.callback(self.on_close_callback)
        buffer = []
        for key in self.bluetooths_dict.keys():
            buffer.append([str(key), str(self.bluetooths_dict[key])])
        for b in buffer:
            self.view.create_bluetooth_entry(b[0],b[1])
        self.view.end(hidden)

    def on_close_callback(self,ptr):
        bluetooths = {}
        for b in self.view.bluetooths:
            name, address = b[0].label(), b[1].label()
            bluetooths[name] = address

        self.model.robot_bluetooth = bluetooths
        #jh = JsonHandler()
        #jh.write(bluetooths,"parameters/bluetooth.json")

        self.view.root.hide()

    def show(self):
        self.view.root.show()



if __name__ == '__main__':
    window_manager = BluetoothManagerController()
