import time

class PID:
    def __init__(self, Kp, Ki, Kd, i_term_min=-float('inf'), i_term_max=float('inf'), target=0.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target = target
        # accumulate integral to speed up computation
        self.i_term = 0.0
        self.i_term_min = i_term_min
        self.i_term_max = i_term_max
        
        self.prev_err = 0.0
        self.prev_time = time.time()

    def update(self, val):
        cur_time = time.time()
        delta_time = cur_time - self.prev_time
        err = self.target - val
        delta_err = err - self.prev_err

        i_term = self.i_term + err * delta_time
        i_term = min(self.i_term_max, max(self.i_term_min, i_term))
        d_term = delta_err / delta_time
        out = self.Kp * err + self.Ki * i_term + self.Kd * d_term

        self.prev_err = err
        self.prev_time = cur_time
        self.i_term = i_term

        return out
