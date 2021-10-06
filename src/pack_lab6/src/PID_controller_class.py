import math
import time


def Hallo():
    print("Hallo")


class PID_controller(object):
    def __init__(self, kp_v, kp_av, ki_v, ki_av, kd_v, kd_av):

        self._kp_v = kp_v
        self._kp_av = kp_av
        self._ki_v = ki_v
        self._ki_av = ki_av
        self._kd_v = kd_v
        self._kd_av = kd_av

        self._ki_v_min = ki_v
        self._ki_av_min = ki_av
        self._ki_v_max = ki_v
        self._ki_av_max = ki_av

    def calcualte_time_loop(self):
        """
        start loop timing and calculate the time of one loop
        first_loop -> dt = 0
        """
        actual_time = time.time()

        if self._last_time == None:
            self._last_time = actual_time

        self.dt = actual_time - self._last_time
        self._last_time = actual_time

    def update_PID_contoller(self, linear_error, angular_error):
        """
        linear_error:= x and y difference between actual position and goal position
        angular_error:= angular difference between actual angel and the angel to the goal

        The actual implementation  of the PID controller.

        """
        self.calcualte_time_loop()

        # P controller part
        linear_velocity_p = self._kp_v * linear_error
        angular_velocity_p = self._kp_av * angular_error

        if(self._linear_error_last == None):
            self._linear_error_last = linear_error
        if(self._angular_error_last == None):
            self._angular_error_last = angular_error

        # D controller part
        print(self.dt)
        linear_velocity_d = self._kd_v * \
            (self._linear_error_last - linear_error)/self.dt
        angular_velocity_d = self._kd_av * \
            (self._linear_error_last - linear_error)/self.dt

        # linear_velocity_i += self.dt*self._kp_v

        self._linear_error_last = linear_error
        self._angular_error_last = angular_error

        self._linear_controler = linear_velocity_p + linear_velocity_d
        self._angular_controler = angular_velocity_p + angular_velocity_d

        return self._linear_controler, self._angular_controler
