import time

class PIDController:

    def __init__(self, kp=1.0, 
                       ki=0.0, 
                       kd=0.0, 
                       max_integral=1000.0, 
                       max_derivative=1000.0, 
                       target=0.0,
                       tolerance=0.0):
        # Constants 
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self._tolerance = tolerance

        # Max integral and derivative value
        self.max_integral = max_integral 
        self.max_derivative = max_derivative

        # Derivative e integral value
        self.derivative = 0.0
        self.integral = 0.0

        # SetPoint
        self.target = target
        self.error = 0.0

        # Initialize time sample 
        self.last_time = time.time()

    def set_target(self, num):
        """Set new target value"""
        self.target = num

    def set_constants(self, kp, ki, kd):
        """
        Update pid constants
        :param kp: float
        :param ki: float
        :param kd: float
        :return:
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def get_constants(self):
        """Return the constant values"""
        return self.kp, self.ki, self.kd    	

    def update(self, value):
        """Update the error value and return as float"""
        error = self.target - value

        timer_aux = time.time()
        delta_time = timer_aux - self.last_time

        # Calculating pid values
        integral = self.integral + (self.error*delta_time)
        derivative = (error - self.error)/delta_time

        # Verify if integral value is greater than max_integral value
        if integral > self.max_integral:
            integral = self.max_integral

        # Verify if derivative value is greater than max_derivative value
        if derivative > self.max_derivative:
            derivative = self.max_derivative

        # Output value
        output = self.kp*error + self.ki*integral + self.kd*derivative

        # Update values
        self.integral = integral
        self.derivative = derivative
        self.error = error
        self.last_time = timer_aux

        if abs(self.error) < self._tolerance:
            return 0
        else:
            return output

    def reset(self):
        """Reset pid values"""
        self.derivative = 0.0
        self.integral = 0.0
        self.error = 0.0
        self.last_time = time.time()