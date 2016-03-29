from collections import deque


class Vector(object):
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def __str__(self):
        return "Vector [{}, {}]".format(self.x, self.y)

    def __iter__(self):
        return tuple([self.x, self.y]).__iter__()

    def __mul__(self, other):
        if isinstance(other, int) or isinstance(other, float):
            return Vector(x=self.x*other, y=self.y*other)
        elif isinstance(other, Vector):
            return Vector(x=self.x*other.x, y=self.y*other.y)

    def pair(self):
        return self.x, self.y


class Pose(object):
    def __init__(self, x, y, theta):
        self.position = Vector(x, y)
        self.orientation = theta

    def __str__(self):
        return "Pose {{position: {0!s}, orientation: {1!s}}}".format(
            self.position, self.orientation)


class Path(object):
    def __init__(self, poses=[]):
        self.poses = deque(poses)

    def __str__(self):
        return "Path {}".format(self.poses)

    def append_pose(self, pose):
        if pose is not None:
            self.poses.append(pose)

    def next_pose(self):
        return self.poses.popleft()


class Twist(object):
    """2D motion described by velocity components

    :ivar roboticsintro.common.Vector linear: linear component of twist
    :ivar float angular: angular component of twist
    """
    def __init__(self, x=0., y=0., theta=0.):
        self.linear = Vector(x, y)
        self.angular = theta

    def __str__(self):
        return "Twist \{linear: {}, angular: {}\}".format(self.linear, self.angular)
