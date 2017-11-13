
MIN_NUM = float('-inf')
MAX_NUM = float('inf')


class PID(object):
    def __init__(self, kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min = mn
        self.max = mx

        self.int_val = self.last_int_val = self.last_error = 0.

    def reset(self):
        self.int_val = 0.0
        self.last_int_val = 0.0

    def step(self, error, sample_time):
        self.last_int_val = self.int_val

	val_p = self.kp * error

        val_i = self.int_val + error * sample_time * self.ki;
	val_i = max(self.min, min(val_i, self.max))

        val_d = (error - self.last_error) / sample_time * self.kd;

        val = val_p + val_i + val_d;

        #val = max(self.min, min(y, self.max))

        if val > self.max:
            val = self.max
        elif val < self.min:
            val = self.min
        else:
            self.int_val = val_i
        self.last_error = error

        return val
