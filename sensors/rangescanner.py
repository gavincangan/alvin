""" A RangeScanner emulates a laser scanner and senses walls, other robots, but not pucks. """

import pyglet
from math import pi, cos, sin
from pymunk import ShapeFilter
from common import *

from configsingleton import ConfigSingleton

class RangeScan:
    """ A scan consists of a predfined list of angles, computed lists of ranges and masks, as well as associated constants. """

    def __init__(self, config_section, robot):

        config = ConfigSingleton.get_instance()
        self.NUMBER_POINTS = config.getint(config_section, "number_points")
        self.ANGLE_MIN = config.getfloat(config_section, "angle_min")
        self.ANGLE_MAX = config.getfloat(config_section, "angle_max")
        self.RANGE_MIN = config.getfloat(config_section, "range_min")
        self.RANGE_MAX = config.getfloat(config_section, "range_max")

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

        #self.INNER_RADIUS = robot.radius + 5
        self.INNER_RADIUS = robot.radius + self.RANGE_MIN
        self.OUTER_RADIUS = self.INNER_RADIUS + self.RANGE_MAX

class RangeScanner:
    def __init__(self, range_scan_sec_name, detection_mask, acceptance_mask):
        # The detection mask is used to indicate all types of objects that
        # the sensor should be sensitive to.  However, if a detected object
        # doesn't also match the acceptance mask then it will be treated as
        # a wall.
        self.detection_mask = detection_mask
        self.acceptance_mask = acceptance_mask

        self.range_scan_sec_name = range_scan_sec_name

    def compute(self, env, robot, visualize=False):
        """ Returns a Scan taken from the given environment and robot. """
        scan = RangeScan(self.range_scan_sec_name, robot)

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
                scan.ranges.append(scan.RANGE_MAX)
                scan.masks.append(0)

                #if visualize:
                #    pyglet.graphics.draw(2, pyglet.gl.GL_LINES,
                #                 ('v2f', (x1, y1, x2, y2)),
                #                 ('c3B', (127, 127, 127, 127, 127, 127)))
            else:
                value = query_info.alpha * scan.RANGE_MAX
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
                    elif object_mask == BLAST_LANDMARK_MASK:
                        pyglet.graphics.draw(2, pyglet.gl.GL_LINES,
                                     ('v2f', (x1, y1, x2, y2)),
                                     ('c3B', (255, 100, 100, 255, 100, 100)))
                    elif object_mask == POLE_LANDMARK_MASK:
                        pyglet.graphics.draw(2, pyglet.gl.GL_LINES,
                                     ('v2f', (x1, y1, x2, y2)),
                                     ('c3B', (100, 100, 255, 100, 100, 255)))
                    elif object_mask == ARC_LANDMARK_MASK:
                        pyglet.graphics.draw(2, pyglet.gl.GL_LINES,
                                     ('v2f', (x1, y1, x2, y2)),
                                     ('c3B', (100, 255, 100, 100, 255, 100)))
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
        
        #print scan.masks
        return scan
