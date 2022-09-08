import time
from rclpy.node import Node

class Watcher:

    ''''Interface simples para sincronizar prints a cada X segundos. '''

    def __init__(self) -> None:
        self.last_time = time.time()
        

    def set_period(self, period: int) -> None:
        self._period = period

    def print(self,node: Node, string: str, period: int = 0.5) -> None:
        '''Imprime uma string desde que a última impressão tenha sido feita a period seconds atrás.'''
        
        cur_time = time.time()

        if cur_time - self.last_time > period:
            node.get_logger().fatal(string)
            self.last_time = cur_time

watcher = Watcher()

__all__ = ["watcher"]