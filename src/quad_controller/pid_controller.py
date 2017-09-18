"""
PIDController class

Author: Jun Zhu
"""


class PIDController(object):
    """"""
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, ki_max=10.0):
        """Initialization

        :param kp: proportional coefficient
        :param ki: integral coefficient
        :param kd: derivative coefficient
        :param ki_max: maximum ki
        """
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._target = None
        self._ki_max = ki_max

        # Store relevant data
        self._last_timestamp = 0.0
        self._start_time = 0.0
        self._integral_error = 0.0
        self._last_error = 0.0

        # Control effort history
        self._max_history_length = 1000
        self._p_history = []
        self._i_history = []
        self._d_history = []

    def reset(self):
        """Reset the controller"""
        self._last_timestamp = 0.0
        self._start_time = 0.0
        self._integral_error = 0.0
        self._last_error = 0.0

        self._p_history = []
        self._i_history = []
        self._d_history = []

    def set_target(self, value):
        """Set controller target value"""
        self._target = value

    def set_kp(self, value):
        """"""
        self._kp = value

    def set_ki(self, value):
        """"""
        if value > self._ki_max:
            self._ki = self._ki_max
        elif value < 0:
            self._ki = 0.0
        else:
            self._ki = value

    def set_kd(self, value):
        """"""
        self._kd = value

    def set_max_ki(self, value):
        """"""
        self._ki_max = value
        if self._ki > self._ki_max:
            self._ki = self._ki_max

    def update(self, measurement, timestamp):
        """Update the controller's state

        :param measurement: float
            Measured value.
        :param timestamp: float
            Time stamp.
        :return: the total control effort
        """
        if self._target is None:
            return 0

        delta_time = timestamp - self._last_timestamp
        if delta_time == 0:
            return 0
        self._last_timestamp = timestamp

        error = self._target - measurement
        self._integral_error += error * delta_time
        delta_error = error - self._last_error
        self._last_error = error

        # Proportional term
        p = self._kp * error

        # Integral term
        i = self._ki * self._integral_error

        # Derivative term
        d = self._kd * delta_error / delta_time

        # Set the control effort
        u = p + i + d

        # add individual control efforts to history
        if len(self._p_history) > self._max_history_length:
            self._p_history.pop(0)
            self._i_history.pop(0)
            self._d_history.pop(0)
        self._p_history.append(p)
        self._i_history.append(i)
        self._d_history.append(d)

        return u


