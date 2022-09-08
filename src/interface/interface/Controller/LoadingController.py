from interface.View.LoadingView import LoadingView
from threading import Thread
from time import sleep

class LoadingController:
    def __init__(self):
        self.load_thread = None

    def start(self, label_text = "Carregando Assets"):
        self.load_thread = LoadingView(label_text)
        self.load_thread.DoRun = True
        self.load_thread.start()

    def stop(self):
        sleep(0.5)
        self.load_thread.DoRun = False
        self.load_thread.join()