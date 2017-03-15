""" A RobotScanner senses only pucks. """

import pyglet
from math import pi, sqrt, atan2
from pymunk import ShapeFilter
from common.angles import normalize_angle_pm_pi

class DetectedRobot:
    def __init__(self, distance, angle):
        self.distance = distance
        self.angle = angle

class RobotScan:
    """ A robot scan is a list of (range, angle) pairs, with associated constants. """
    MAX_RANGE = 1200 
    MIN_RANGE = 0 
    MAX_ANGLE = pi/30.0
    MIN_ANGLE = -pi/30.0
    #MAX_ANGLE = pi/2.0
    #MIN_ANGLE = -pi/2.0

    def __init__(self):
        self.robots = []

class RobotScanner:
    def compute(self, env, this_robot, others, visualize=False):
        """ Returns a PuckScan taken from the given environment, this robot,
            and list of other robots. """

        scan = RobotScan()

        if visualize:
            pyglet.gl.glLineWidth(4)

        for bot in others:
            dx = bot.body.position.x - this_robot.body.position.x
            dy = bot.body.position.y - this_robot.body.position.y
            distance = sqrt(dx*dx + dy*dy)
            angle = normalize_angle_pm_pi(atan2(dy, dx) - this_robot.body.angle)
            if distance <= scan.MAX_RANGE and distance >= scan.MIN_RANGE and \
               angle <= scan.MAX_ANGLE and angle >= scan.MIN_ANGLE:
                scan.robots.append(DetectedRobot(distance, angle))

                if visualize:
                    x1 = this_robot.body.position.x
                    y1 = this_robot.body.position.y
                    x2 = bot.body.position.x
                    y2 = bot.body.position.y
                    pyglet.graphics.draw(2, pyglet.gl.GL_LINES,
                                 ('v2f', (x1, y1, x2, y2)),
                                 ('c3B', (0, 255, 255, 0, 255, 255)))

        if visualize:
            pyglet.gl.glLineWidth(1)
        return scan
