from math import pi, floor

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
