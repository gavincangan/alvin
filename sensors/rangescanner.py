""" A RangeScanner emulates a laser scanner and senses walls, other robots, but not pucks. """

import pyglet
from math import pi, cos, sin
from pymunk import ShapeFilter
from common import WALL_MASK, ROBOT_MASK, RED_PUCK_MASK, GREEN_PUCK_MASK, BLUE_PUCK_MASK

class RangeScan:
    """ A scan consists of a predfined list of angles, computed lists of ranges and masks, as well as associated constants. """

    """
    NUMBER_POINTS = 50 
    ANGLE_MIN = -pi/2.0
    ANGLE_MAX = pi/2.0
    """
    NUMBER_POINTS = 10
    ANGLE_MIN = -pi/4
    ANGLE_MAX = pi/4
    MIN_VALUE = 0
    MAX_VALUE = 1000

    def __init__(self, robot):

        self.angles = []
        if self.NUMBER_POINTS == 1:
            self.angles.append(self.ANGLE_MIN)
        else:
            angle_delta = (self.ANGLE_MAX - self.ANGLE_MIN) / \
                          (self.NUMBER_POINTS - 1)
            for i in range(self.NUMBER_POINTS):
                self.angles.append(self.ANGLE_MIN + i * angle_delta)

        self.ranges = []
        self.masks = []

        self.INNER_RADIUS = robot.radius + 5
        self.OUTER_RADIUS = self.INNER_RADIUS + self.MAX_VALUE

class RangeScanner:
    def __init__(self, detection_mask, acceptance_mask):
        # The detection mask is used to indicate all types of objects that
        # the sensor should be sensitive to.  However, if a detected object
        # doesn't also match the acceptance mask then it will be treated as
        # a wall.
        self.detection_mask = detection_mask
        self.acceptance_mask = acceptance_mask

    def compute(self, env, robot, visualize=False):
        """ Returns a Scan taken from the given environment and robot. """
        scan = RangeScan(robot)

        for sensor_angle in scan.angles:
            c = cos(robot.body.angle + sensor_angle)
            s = sin(robot.body.angle + sensor_angle)
            x1 = int(robot.body.position.x + scan.INNER_RADIUS * c)
            y1 = int(robot.body.position.y + scan.INNER_RADIUS * s)
            x2 = int(robot.body.position.x + scan.OUTER_RADIUS * c)
            y2 = int(robot.body.position.y + scan.OUTER_RADIUS * s)

            #mask = ShapeFilter.ALL_MASKS ^ 2
            shape_filter = ShapeFilter(mask=self.detection_mask)
            query_info = env.segment_query_first((x1, y1), (x2, y2), 1, \
                                                 shape_filter)
            if query_info == None or query_info.shape == None:
                scan.ranges.append(scan.MAX_VALUE)
                scan.masks.append(0)

                if visualize:
                    pyglet.graphics.draw(2, pyglet.gl.GL_LINES,
                                 ('v2f', (x1, y1, x2, y2)),
                                 ('c3B', (127, 127, 127, 127, 127, 127)))
            else:
                value = query_info.alpha * scan.MAX_VALUE
                object_mask = query_info.shape.filter.categories
                if object_mask & self.acceptance_mask == 0:
                    # The detected shape is not accepted, we will treat
                    # it as a wall.
                    object_mask = WALL_MASK

                scan.ranges.append(value)
                scan.masks.append(object_mask)

                if visualize:
                    x2 = int(robot.body.position.x + \
                             (scan.INNER_RADIUS + value) * c)
                    y2 = int(robot.body.position.y + \
                             (scan.INNER_RADIUS + value) * s)
                    if object_mask == WALL_MASK:
                        pyglet.graphics.draw(2, pyglet.gl.GL_LINES,
                                     ('v2f', (x1, y1, x2, y2)),
                                     ('c3B', (255, 255, 0, 255, 255, 0)))
                        #pass
                    elif object_mask == ROBOT_MASK:
                        pyglet.graphics.draw(2, pyglet.gl.GL_LINES,
                                     ('v2f', (x1, y1, x2, y2)),
                                     ('c3B', (0, 255, 255, 0, 255, 255)))
                    elif object_mask == RED_PUCK_MASK:
                        pyglet.graphics.draw(2, pyglet.gl.GL_LINES,
                                     ('v2f', (x1, y1, x2, y2)),
                                     ('c3B', (255, 0, 0, 255, 0, 0)))
                    elif object_mask == GREEN_PUCK_MASK:
                        pyglet.graphics.draw(2, pyglet.gl.GL_LINES,
                                     ('v2f', (x1, y1, x2, y2)),
                                     ('c3B', (0, 255, 0, 0, 255, 0)))
                    elif object_mask == BLUE_PUCK_MASK:
                        pyglet.graphics.draw(2, pyglet.gl.GL_LINES,
                                     ('v2f', (x1, y1, x2, y2)),
                                     ('c3B', (0, 0, 255, 0, 0, 255)))
        return scan
