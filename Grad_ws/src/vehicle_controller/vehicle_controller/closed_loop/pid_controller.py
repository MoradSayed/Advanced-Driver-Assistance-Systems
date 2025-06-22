class PID:
    def __init__(self, KP:float, KI:float, KD:float, target:int|float, lim:tuple[int, int]):
        self.kp = KP
        self.ki = KI
        self.kd = KD

        self.set_point = target
        self.saturation_min, self.saturation_max = lim
        
        self.error_last = 0
        self.integral_error = 0
        self.last_time = None

    def compute(self, measured_value, current_time):
        if self.last_time:
            dt = current_time - self.last_time
            if dt == 0:
                dt = 0.0001
        else:
            dt = 0.1

        error = self.set_point - measured_value   # compute the error
        derivative_error = (error - self.error_last) / dt   # find the derivative of the error (how the error changes with time)
        self.integral_error += error * dt   # error build up over time

        output = self.kp*error + self.ki*self.integral_error + self.kd*derivative_error
        self.error_last = error

        if output > self.saturation_max and self.saturation_max is not None:
            output = self.saturation_max
        elif output < self.saturation_min and self.saturation_min is not None:
            output = self.saturation_min

        self.last_time = current_time
        return output
