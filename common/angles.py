from math import floor

def normalize_angle(theta):
    """Convert angle into positive value in [0,2Pi)

    :param float theta: angle
    """
    TWO_PI = 2 * pi
    return theta - TWO_PI * floor(theta / TWO_PI)
