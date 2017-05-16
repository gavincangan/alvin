from math import fabs, floor, pi

def normalize_angle_0_2pi(theta):
    """Convert angle into positive value in [0,2pi)

    :param float theta: angle
    """
    TWO_PI = 2 * pi
    return theta - TWO_PI * floor(theta / TWO_PI)

def normalize_angle_pm_pi(theta):
    """Convert angle into range [-pi,pi)

    :param float theta: angle
    """
    TWO_PI = 2 * pi
    while theta >= pi:
        theta -= TWO_PI
    while theta < -pi:
        theta += TWO_PI
    return theta

def get_smallest_angular_difference(a, b):
    a = normalize_angle_pm_pi(a)
    b = normalize_angle_pm_pi(b)
    error = fabs(a - b)
    if error > pi:
        error = fabs(error - 2*pi)
    return error

def get_angular_difference(a, b):
    a = normalize_angle_0_2pi(a)
    b = normalize_angle_0_2pi(b)
    error = a - b
    if error < 0:
        error = error + 2*pi
    return error
