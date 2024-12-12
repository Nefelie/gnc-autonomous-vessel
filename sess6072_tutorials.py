# import matplotlib.pyplot as plt
# import matplotlib.dates as mdate
# from matplotlib import cm
# import numpy as np
# import netCDF4
# from scipy.interpolate import interp1d, interp2d, RectBivariateSpline
# from math import sqrt, pi, cos, sin
# from mpl_toolkits.mplot3d import Axes3D
# from matplotlib.patches import Ellipse

GRAVITY = 9.81


def clamp(value, limits):
    """Clamps an input value using a tuple of limits

    Parameters
    ----------
    value : float
        The value to clamp.
    limits : tuple(float, float)
        The lower and upper limits.

    Returns
    -------
    float
        Returns the same value if between limits, returns the limit if out of boundaries.
    """
    lower, upper = limits
    if value is None:
        return None
    elif upper is not None and value > upper:
        return upper
    elif lower is not None and value < lower:
        return lower
    return value


class PID:
    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0, setpoint=0, limits=(None, None)):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.setpoint = setpoint
        self.previous_input = 0
        self.integral = 0
        self.limits = limits

    def __call__(self, measured_value, dt=0.5):
        error = self.setpoint - measured_value
        proportional = self.Kp * error
        self.integral += self.Ki * error * dt
        self.integral = clamp(self.integral, self.limits)
        derivative = -self.Kd * (measured_value - self.previous_input) / dt
        output = clamp(proportional + self.integral + derivative, self.limits)
        self.previous_input = measured_value
        return output

