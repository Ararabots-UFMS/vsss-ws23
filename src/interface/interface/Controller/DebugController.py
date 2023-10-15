# -*- coding: utf-8 -*-
from interface.View.DebugView import DebugView
import fltk as fl
from utils.yaml_handler import YamlHandler


class DebugController():
    def __init__(self, _debug_params, hidden=False):
        self.debug_params = _debug_params
        #_debug_params is a dict like: {"movement_predict_simulation":True, "robot_vector":True,
        #"things":{"robot_1":True, "robot_2":True, "robot_3":true,"robot_4":True, "robot_5":True,"ball":True}}
        self.view = DebugView()
        self.faster_hash = ['robot_'+str(x) for x in range(1, 6)]        
        self.faster_hash.append("ball")
        
        self.view.check_simulation.value(self.debug_params["movement_predict_simulation"])
        self.view.check_robot_vector.value(self.debug_params["robot_vector"])
        #logfatal(str(self.debug_params["robot_vector"]))

        for i in range(len(self.view.check_robots)):
            self.view.check_robots[i].value(self.debug_params["things"][self.faster_hash[i]])

        self.debug_params = _debug_params
        self.view.root.callback(self.on_close_callback)        
        self.view.end(hidden)

    def on_close_callback(self,ptr):
        self.debug_params["movement_predict_simulation"] = self.view.check_simulation.value()
        self.debug_params["robot_vector"] = self.view.check_robot_vector.value()        
        for i,f in enumerate(self.faster_hash):
            self.debug_params["things"][f] = self.view.check_robots[i].value()

        self.view.root.hide()

    def show(self):
        self.view.root.show()


if __name__ == '__main__':
    window_manager = DebugController()
