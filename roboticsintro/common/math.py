from __future__ import absolute_import

from math import cos, sin, pi, floor
from ..common import Vector


def unit_direction_vector(theta=0.0):
    """Calculate unit direction vector from angle.

    :param theta: angle from positive x-axis, positive counter-clockwise
    :type theta: float radians
    :returns: unit vector at angle theta from positive x-axis
    :rtype: roboticsintro.common.Vector
    """
    return Vector(x=cos(theta), y=sin(theta))


def vector_angle(x, y):
    """Calculate angle from x-axis of vector.

    :param x: x component of vector
    :type x: float
    :param y: y component of vector
    :type y: float
    :returns: angle in radians from x-axis
    :rtype: float
    """
    pass


def normalize_angle(theta):
    """Convert angle into positive value in [0,2Pi)

    :param float theta: angle
    """
    TWO_PI = 2 * pi
    return theta - TWO_PI * floor(theta / TWO_PI)
