import rospy

MIN_NUM = float('-inf')
MAX_NUM = float('inf')


class PID(object):
    def __init__(self, kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min = mn
        self.max = mx

        self.int_val = self.last_error = 0.

    def reset(self):
        self.int_val = 0.0

    def step(self, error, sample_time):
        # error is negative if current value is smaller than target value
        integral = self.int_val + error * sample_time
        derivative = (error - self.last_error) / sample_time

        val = self.kp * error + self.ki * integral + self.kd * derivative

        if val > self.max:
            val = self.max
        elif val < self.min:
            val = self.min
        else:
            self.int_val = integral
        self.last_error = error

        rospy.logwarn("---PID Controler---")
        rospy.logwarn("Error: {0}      \n".format(error))
        rospy.logwarn("Integral: {0}   \n".format(integral))
        rospy.logwarn("Derivative: {0} \n".format(derivative))
        rospy.logwarn("---------------------------------")

        return val
