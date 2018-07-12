""" A LandmarkScanner senses only landmarks. """

import pyglet
from math import pi, sqrt, atan2
from common.angles import normalize_angle_pm_pi
from configsingleton import ConfigSingleton

class DetectedLandmark:
    def __init__(self, distance, angle):
        self.distance = distance
        self.angle = angle

class LandmarkScan:
    """ A landmark scan is a list of (range, angle) pairs, with associated constants. """
    def __init__(self):

        config = ConfigSingleton.get_instance()
        self.MIN_ANGLE = config.getfloat("LandmarkScan", "min_angle")
        self.MAX_ANGLE = config.getfloat("LandmarkScan", "max_angle")
        self.MIN_RANGE = config.getfloat("LandmarkScan", "min_range")
        self.MAX_RANGE = config.getfloat("LandmarkScan", "max_range")

        self.landmarks = []

class LandmarkScanner:
    def compute(self, env, robot, landmarks, visualize=False):
        """ Returns a LandmarkScan taken from the given environment, robot, and 
        list of landmarks. """

        scan = LandmarkScan()

        if visualize:
            pyglet.gl.glLineWidth(3)

        for landmark in landmarks:
            dx = landmark.body.position.x - robot.body.position.x
            dy = landmark.body.position.y - robot.body.position.y
            distance = sqrt(dx*dx + dy*dy)
            angle = normalize_angle_pm_pi(atan2(dy, dx) - robot.body.angle)
            if distance <= scan.MAX_RANGE and distance >= scan.MIN_RANGE and \
               angle <= scan.MAX_ANGLE and angle >= scan.MIN_ANGLE:
                scan.landmarks.append(DetectedLandmark(distance, angle))

                if visualize:
                    x1 = robot.body.position.x
                    y1 = robot.body.position.y
                    x2 = landmark.body.position.x
                    y2 = landmark.body.position.y
                    pyglet.graphics.draw(2, pyglet.gl.GL_LINES,
                                 ('v2f', (x1, y1, x2, y2)),
                                 ('c3B', (200, 200, 255, 200, 200, 255)))

        if visualize:
            pyglet.gl.glLineWidth(1)
        return scan
