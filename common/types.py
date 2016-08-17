import pyglet

class Twist(object):
    """2D motion described by velocity components

    :ivar roboticsintro.common.Vector linear: linear component of twist
    :ivar float angular: angular component of twist (z-axis rotation)
    """
    def __init__(self, linear=0., angular=0.):
        self.linear = linear
        self.angular = angular

    def __str__(self):
        return "Twist {{linear: {}, angular: {}}}".format(
            self.linear, self.angular)
