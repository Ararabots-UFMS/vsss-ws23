import rclpy
from rclpy.node import Node

class Logger:
    def __init__(self, node: Node):
        self.last_fatal_log = ''
        self.last_warn_log = ''
        self._logger = node.get_logger()

    def log_fatal(self, log):
        if log != self.last_fatal_log:
            self._logger.fatal(log)
            self.last_fatal_log = log

    def log_warn(self, log):
        if log != self.last_warn_log:
            self._logger.warn(log)
            self.last_warn_log = log