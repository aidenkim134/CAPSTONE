import time

class PID:
    """PID Controller
    """

    def __init__(self, P=1, I=1.2, D=0.1, set_point=None):

        self.Kp = P; self.Ki = I; self.Kd = D

        self.sample_time = 0.1
        self.current_time = time.time()
        self.last_time = self.current_time
        self.SetPoint = set_point

        self.clear()

    def clear(self):
        """Clears PID computations and coefficients"""


        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        self.output = 0.0

    def update(self, feedback_value):
        """
        Calculates PID value for given reference feedback

        u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}
        """
        error = self.SetPoint - feedback_value

        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time * self.Ki

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = (delta_error / delta_time) * self.Kd

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

            self.output = self.PTerm + self.ITerm + self.DTerm
            if self.output > 100:
                self.output = 100
            if self.output < 0:
                self.output = 0
            
            
    
    def setSampleTime(self, sample_time):
        self.sample_time = sample_time


if __name__ == '__main__':
    w_PID = PID(set_point = 40)
    for i in range(1,20):
        time.sleep(0.1)

        w_PID.update(40*i / 20)
        print(w_PID.output)


