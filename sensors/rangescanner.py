import pyglet
from math import pi, cos, sin

class Scan:
    """ A scan is a list of (range, angle) pairs, with associated constants. """
    NUMBER_POINTS = 50 
    ANGLE_MIN = -pi/2.0
    ANGLE_MAX = pi/2.0

    def __init__(self, robot):
        self.angles = []
        angle_delta = (self.ANGLE_MAX - self.ANGLE_MIN)/(self.NUMBER_POINTS - 1)
        for i in range(self.NUMBER_POINTS):
            self.angles.append(self.ANGLE_MIN + i * angle_delta)
        self.ranges = []

        self.MIN_VALUE = 0
        self.MAX_VALUE = 200
        self.INNER_RADIUS = robot.radius + 5
        self.OUTER_RADIUS = self.INNER_RADIUS + self.MAX_VALUE

class RangeScanner:
    def compute(self, env, robot, visualize=False):
        """ Returns a Scan taken from the given environment and robot. """
        scan = Scan(robot)

        for sensor_angle in scan.angles:
            c = cos(robot.body.angle + sensor_angle)
            s = sin(robot.body.angle + sensor_angle)
            x1 = int(robot.body.position.x + scan.INNER_RADIUS * c)
            y1 = int(robot.body.position.y + scan.INNER_RADIUS * s)
            x2 = int(robot.body.position.x + scan.OUTER_RADIUS * c)
            y2 = int(robot.body.position.y + scan.OUTER_RADIUS * s)

            query_info = env.segment_query_first((x1, y1), (x2, y2), 1, [])
            if query_info == None or query_info.shape == None:
                scan.ranges.append(scan.MAX_VALUE)

                if visualize:
                    pyglet.graphics.draw(2, pyglet.gl.GL_LINES,
                                 ('v2f', (x1, y1, x2, y2)),
                                 ('c3B', (0, 255, 0, 0, 255, 0)))
            else:
                value = query_info.alpha * scan.MAX_VALUE
                scan.ranges.append(value)

                if visualize:
                    x2 = int(robot.body.position.x + \
                             (scan.INNER_RADIUS + value) * c)
                    y2 = int(robot.body.position.y + \
                             (scan.INNER_RADIUS + value) * s)
                    pyglet.graphics.draw(2, pyglet.gl.GL_LINES,
                                 ('v2f', (x1, y1, x2, y2)),
                                 ('c3B', (255, 0, 0, 255, 0, 0)))
        return scan
