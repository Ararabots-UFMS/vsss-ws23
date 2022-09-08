from time import time
from typing import Union, Tuple, List

from robot.comunication.sender import STDMsg, SelfControlMsg

# TODO: Debugar isso aqui completamente

class RobotHardware:
    def __init__(self, max_speed: int = 255):
        self._hardware_max_speed = max_speed
        self._max_enable_speed = 70
        self._allowed_speed = self._max_enable_speed
        self._last_step_time = time()
        self._step_time = 0.05
        self._speed_step = 15

        self.LEFTFORWARD_RIGHTFORWARD = 0x00  # 0000 0000
        self.LEFTFORWARD_RIGHTBACKWARD = 0x01  # 0000 0001
        self.LEFTBACKWARD_RIGHTFORWARD = 0x02  # 0000 0010
        self.LEFTBACKWARD_RIGHTBACKWARD = 0x03  # 0000 0011

        self.PID_ON_HARDWARE_OP_CODE = 0x00     # 0001 0000 

        self._current_motors_direction = self.LEFTFORWARD_RIGHTFORWARD

    def encode(self, msg: Union[STDMsg, SelfControlMsg]) -> List:
        if isinstance(msg, STDMsg):
            message = self.encode_std_msg(msg)
        elif isinstance(msg, SelfControlMsg):
            message = self.encode_self_control(msg)

        return message

    def encode_std_msg(self, msg: STDMsg) -> List[int]:
        message = []
        
        self.set_current_direction(msg.left_speed, msg.right_speed)
        message.append(self._current_motors_direction)
        
        left, right = int(msg.left_speed), int(msg.right_speed)
        message.append(abs(left))
        message.append(abs(right))

        return message

    def encode_self_control(self, msg: SelfControlMsg) -> List[int]:
        message = []
        message.append(self.PID_ON_HARDWARE_OP_CODE | msg.direction)
        message.append(msg.delta_theta)
        message.append(msg.speed)
        
        return message
        

    def set_current_direction(self, left_speed: float, right_speed: float) -> None:
        if left_speed >= 0 and right_speed >= 0:
            self._current_motors_direction = self.LEFTFORWARD_RIGHTFORWARD
        elif left_speed >= 0 and right_speed <= 0:
            self._current_motors_direction = self.LEFTFORWARD_RIGHTBACKWARD
        elif left_speed <= 0 and right_speed >= 0:
            self._current_motors_direction = self.LEFTBACKWARD_RIGHTFORWARD
        else:
            self._current_motors_direction = self.LEFTBACKWARD_RIGHTBACKWARD

    def normalize_speeds(self, msg: STDMsg) -> STDMsg:
        old_direction = self._current_motors_direction
        self.set_current_direction(msg.left_speed, msg.right_speed)
        if old_direction != self._current_motors_direction:
            self._allowed_speed = self._max_enable_speed

        max_abs_speed = max(abs(msg.left_speed), abs(msg.right_speed))
        self.update_allowed_speed(max_abs_speed)
        
        left, right = self.limit_output(msg.left_speed, msg.right_speed)
        return STDMsg(left, right)

    def update_allowed_speed(self, target_abs_speed: float) -> None:

        t = time()

        if target_abs_speed > self._hardware_max_speed:
            target_abs_speed = self._hardware_max_speed

        if target_abs_speed < self._allowed_speed:
            self._allowed_speed = target_abs_speed
            self._last_step_time = t
        else:
            if self._allowed_speed < self._max_enable_speed:
                self._allowed_speed = min(self._max_enable_speed, target_abs_speed)
                self._last_step_time = t
            elif t - self._last_step_time > self._step_time:
                self._allowed_speed = min(target_abs_speed,
                                          self._allowed_speed + self._speed_step)
                self._last_step_time = t

    def get_next_speed(self):
        return min(self._allowed_speed + self._speed_step, 256)

    def limit_output(self, left_speed: float, right_speed: float) -> Tuple[int, int]:
        if abs(left_speed) > self._allowed_speed or \
                abs(right_speed) > self._allowed_speed:
            max_ = max(abs(left_speed), abs(right_speed))
            inv = 1.0 / max_

            left_speed = inv * left_speed * self._allowed_speed
            right_speed = inv * right_speed * self._allowed_speed

            return int(left_speed), int(right_speed)
        else:
            return int(left_speed), int(right_speed)

class SimulationHardware(RobotHardware):

    # TODO: determinar o que fazer de diferente aqui
    # a simulação espera velocidades em radianos por segundo
    # velocidade máxima testada: 1.5 m/s
    # Raio das rodas do robô Parsian: 0.02
    # Matemágica w_max = 1.5 / 0.02 = 75 rad/s
    # Podemos "supor" que a w_max seja em torno de 100 rad/s
    # "Teoricamente", não importa o limite de velocidade, podemos deixar o 255

    def __init__(self, max_speed: float = 100):
        super().__init__(max_speed=max_speed)

        # Atualizando parâmetros
        # self._max_enable_speed = max_speed
        self._max_enable_speed = 75
        self._allowed_speed = self._max_enable_speed

        self._step_time = 0.0
        self._speed_step = 255 # aceleração

    def encode_std_msg(self, msg: STDMsg) -> List[int]:
        message = []
        
        self.set_current_direction(msg.left_speed, msg.right_speed)
        message.append(self._current_motors_direction)
        
        left, right = msg.left_speed, msg.right_speed
        message.append(abs(left))
        message.append(abs(right))

        return message

    def get_next_speed(self):
        return min(self._allowed_speed + self._speed_step, 100)


    def update_allowed_speed(self, target_abs_speed: float) -> None:

        self._allowed_speed = 100

    def limit_output(self, left_speed: float, right_speed: float) -> Tuple[int, int]:
        # Não precisamos mais que as velocidades sejam int

        if abs(left_speed) > self._allowed_speed or \
                abs(right_speed) > self._allowed_speed:
            max_ = max(abs(left_speed), abs(right_speed))
            inv = 1.0 / max_

            left_speed = inv * left_speed * self._allowed_speed
            right_speed = inv * right_speed * self._allowed_speed

            return left_speed, right_speed
        else:
            return left_speed, right_speed