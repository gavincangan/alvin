""" A PuckScanner senses only pucks. """

import pyglet
from math import pi, sqrt, atan2
from pymunk import ShapeFilter
from common.angles import normalize_angle_pm_pi

class DetectedPuck:
    def __init__(self, distance, angle, kind):
        self.distance = distance
        self.angle = angle
        self.kind = kind

class PuckScan:
    """ A puck scan is a list of (range, angle) pairs, with associated constants. """
    MAX_RANGE = 1200 
    MIN_RANGE = 0 
    MAX_ANGLE = pi/30.0
    MIN_ANGLE = -pi/30.0
    #MAX_ANGLE = pi/2.0
    #MIN_ANGLE = -pi/2.0

    def __init__(self):
        self.pucks = []

class PuckScanner:
    def compute(self, env, robot, pucks, visualize=False):
        """ Returns a PuckScan taken from the given environment, robot, and 
        list of pucks. """

        scan = PuckScan()

        if visualize:
            pyglet.gl.glLineWidth(3)

        for puck in pucks:
            dx = puck.body.position.x - robot.body.position.x
            dy = puck.body.position.y - robot.body.position.y
            distance = sqrt(dx*dx + dy*dy)
            angle = normalize_angle_pm_pi(atan2(dy, dx) - robot.body.angle)
            if distance <= scan.MAX_RANGE and distance >= scan.MIN_RANGE and \
               angle <= scan.MAX_ANGLE and angle >= scan.MIN_ANGLE:
                scan.pucks.append(DetectedPuck(distance, angle, puck.kind))

                if visualize:
                    x1 = robot.body.position.x
                    y1 = robot.body.position.y
                    x2 = puck.body.position.x
                    y2 = puck.body.position.y
                    pyglet.graphics.draw(2, pyglet.gl.GL_LINES,
                                 ('v2f', (x1, y1, x2, y2)),
                                 ('c3B', (255, 255, 0, 255, 255, 0)))

        if visualize:
            pyglet.gl.glLineWidth(1)
        return scan
