from .fuzzy_pid import Fuzzy

class PID:
    def __init__(self, KP:float, KI:float, KD:float, target:int|float, lim:tuple[float, float]=(float("-inf"), float("inf")), fuzzy : Fuzzy = None):
        self.kp = KP
        self.ki = KI
        self.kd = KD

        self.set_point = target
        self.saturation_min, self.saturation_max = lim
        
        self.fuzz_comp = fuzzy.compute if fuzzy != None else lambda *args: (1, 1, 1)
        self.reset()

    def compute(self, measured_value, current_time, max_error, max_delta_error=10):
        dt = max((current_time - self.prev_time) if self.prev_time else 0.05, 0.05)
        self.prev_time = current_time

        error = measured_value - self.set_point             # compute the error
        self.integral_error += error * dt                   # error build up over time
        derivative_error = (error - self.error_last) / dt   # find the derivative of the error (how the error changes with time)
        self.error_last = error

        kp_mod, ki_mod, kd_mod = self.fuzz_comp(error, max_error, derivative_error, max_delta_error)

        output = (self.kp*error)*kp_mod + (self.ki*self.integral_error)*ki_mod + (self.kd*derivative_error)*kd_mod
        output = max(self.saturation_min, min(output, self.saturation_max))

        return output

    def reset(self):
        self.error_last = 0
        self.integral_error = 0
        self.prev_time = None
