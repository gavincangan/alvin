""" Variant on Gauci et al's controller for object clustering. """

import pyglet
from controller import Controller
from math import fabs, pi
from random import random
from common import Twist, WALL_MASK, ROBOT_MASK, M_TO_PIXELS, RED_PUCK_MASK, GREEN_PUCK_MASK, BLUE_PUCK_MASK
from common.angles import normalize_angle_pm_pi

class MyController(Controller):

    def __init__(self, acceptable_puck_mask):
        self.acceptable_puck_mask = acceptable_puck_mask
        self.current_puck_type = None

    def draw_circle(self, robot, red, green, blue, thickness):
        x1 = robot.body.position.x - 2
        y1 = robot.body.position.y - 2
        x2 = robot.body.position.x + 2
        y2 = robot.body.position.y + 2
        pyglet.gl.glLineWidth(thickness)
        vertices = (x1, y1, x2, y1, x2, y2, x1, y2)
        colors = (red, green, blue, red, green, blue, \
                  red, green, blue, red, green, blue)
        pyglet.graphics.draw(4, pyglet.gl.GL_QUADS,
            ('v2f', vertices),
            ('c3B', colors))
        pyglet.gl.glLineWidth(1)

    def react(self, this_robot, sensor_suite, visualize=False):
        twist = Twist()

        scan = sensor_suite.range_scan

        # By default we will attend to the centre of the scan
        n = len(scan.ranges)
        centre_index = n/2

        # ...but we will shift attention based on the compass
        # John: Uncomment to play with compass-modulated shifting.
        """
        compass_angle = normalize_angle_pm_pi(this_robot.body.angle)
        sign_dir = 1
        if self.current_puck_type != None and (self.current_puck_type & RED_PUCK_MASK) != 0:
            sign_dir = -1
        if compass_angle < 0:
            centre_index = sign_dir * 2
        else:
            centre_index = -sign_dir * 2
        """

        puck_ahead = (scan.masks[centre_index] & self.acceptable_puck_mask) != 0
        puck_mask = scan.masks[centre_index]

        # Set current_puck_type if uninitialized.
        if self.current_puck_type == None and puck_ahead:
            self.current_puck_type = puck_mask

        # With a small random probability, change current_puck_type to match
        # the puck ahead.
        # John: Uncomment to play with sorting
        """
        if puck_ahead and random() < 0.005:
            self.current_puck_type = puck_mask
        """

        # Set the two predicates 'react_to_puck' and 'react_to_robot'.  Only
        # one should be true.
        react_to_puck = puck_ahead and puck_mask == self.current_puck_type

        react_to_robot = scan.masks[centre_index] == ROBOT_MASK
        assert not (react_to_puck and react_to_robot)

        # Now react...
        if react_to_puck:
            # Turn right
            twist.linear = 4
            twist.angular =  2.0
        elif react_to_robot:
            # Turn left and slow
            twist.linear = 0.5 
            twist.angular = -2.0
        else:
            # Turn left
            twist.linear = 4
            twist.angular = -2.0

        if visualize:
            """
            if self.current_puck_type == RED_PUCK_MASK:
                self.draw_circle(this_robot, 255, 0, 0, 1)
            elif self.current_puck_type == GREEN_PUCK_MASK:
                self.draw_circle(this_robot, 0, 255, 0, 1)
            elif self.current_puck_type == BLUE_PUCK_MASK:
                self.draw_circle(this_robot, 0, 0, 255, 1)
            else:
                self.draw_circle(this_robot, 0, 255, 255, 255)
            """
            if react_to_puck:
                self.draw_circle(this_robot, 255, 0, 0, 1)
            elif react_to_robot:
                self.draw_circle(this_robot, 0, 0, 255, 1)
            else:
                self.draw_circle(this_robot, 0, 255, 0, 1)

        return twist
