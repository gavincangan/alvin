""" Controller to build walls using pairs of landmarks with a low-level controller inspired by Gauci et al's concept, as re-interpreted by LeftmostController.
"""

import pyglet
from controller import Controller
from math import fabs, pi, sin, cos
from random import random
from common import Twist, WALL_MASK, ANY_LANDMARK_MASK, ROBOT_MASK, M_TO_PIXELS, RED_PUCK_MASK, GREEN_PUCK_MASK, BLUE_PUCK_MASK
from common.angles import normalize_angle_pm_pi
from configsingleton import ConfigSingleton

class LandmarkWallController(Controller):

    def __init__(self, acceptable_puck_mask):
        self.acceptable_puck_mask = acceptable_puck_mask

        #config = ConfigSingleton.get_instance()
        #self.attend_to_landmarks = config.getboolean("LandmarkWallController", "attend_to_landmarks")

    def get_all_landmarks_leftmost_indices(self, lscan):

        # First find the index of the leftmost landmark
        left_index = None
        n = len(lscan.ranges)
        for i in range(n-1, -1, -1):
            if lscan.masks[i] & ANY_LANDMARK_MASK != 0:
                left_index = i
                break
        print left_index

        # Now, "get through" the rest of the leftmost landmark pixels (if any)
        non_lmark_index = None
        for i in range(left_index, -1, -1):
            if lscan.masks[i] & ANY_LANDMARK_MASK == 0:
                non_lmark_index = i
                break

        # Now get the leftmost pixel of the next landmark
        right_index = None
        for i in range(non_lmark_index, -1, -1):
            if lscan.masks[i] & ANY_LANDMARK_MASK != 0:
                right_index = i
                break
            
        return (left_index, right_index)


    def get_leftmost_puck_between_landmarks(self, rscan, (left_index, right_index)):
        index = None
        n = len(rscan.ranges)
        for i in range(left_index-1, right_index, -1):
            if rscan.masks[i] & self.acceptable_puck_mask != 0:
                left_index = i
                break
            

    def react(self, this_robot, sensor_suite, visualize=False):
        twist = Twist()

        rscan = sensor_suite.range_scan
        lscan = sensor_suite.landmark_scan
        n = len(rscan.ranges)

        (left, right) = self.get_closest_landmark_pair_leftmost_indices(lscan)
        (left_angle, right_angle) = (self.index_to_angle(lcan, left), self.index_to_angle(lcan, left))

        self.draw_line(this_robot, rscan, left_angle, (255, 0, 0))
        self.draw_line(this_robot, rscan, right_angle, (0, 255, 0))

#
#    def react(self, this_robot, sensor_suite, visualize=False):
#        twist = Twist()
#
#        rscan = sensor_suite.range_scan
#        lscan = sensor_suite.landmark_scan
#        n = len(rscan.ranges)
#
#        (left, right) = self.get_closest_landmark_pair_leftmost_indices(lscan)
#
#        angle_pair = self.get_closest_landmark_pair_leftmost_angles(lscan)
#        assert angle_pair != None and angle_pair.count(None) == 0
#
#        target_angle = self.get_leftmost_puck_within_angle_pair(rscan, 
#                                                                angle_pair)
#
#        if target_angle == None:
#            # Graze the leftmost landmark
#            target_angle = angle_pair[0] if angle_pair[0] > angle_pair[1] else angle_pair[1]
#
#        # Check for the presence of another robot in the centre of the image.
#        react_to_robot = False
#        react_to_robot_angle = None
#        for i in range(0, n):
#            if (rscan.masks[i] == ROBOT_MASK and fabs(rscan.angles[i]) < pi/4 and
#                rscan.ranges[i] < 2*this_robot.radius):
#                react_to_robot = True
#                react_to_robot_angle = (rscan.ANGLE_MIN + i * 
#                    (rscan.ANGLE_MAX - rscan.ANGLE_MIN) / float(rscan.NUMBER_POINTS))
#
#        if react_to_robot:
#            # Turn left and slow
#            twist.linear = 0.5 
#            twist.angular = 2.0
#
#            if visualize: # Reacting to robot
#                self.draw_line(this_robot, rscan, react_to_robot_angle,
#                               (255, 0, 0))
#
#        elif (target_angle == None or target_angle <= 0):
#            # Turn right
#            twist.linear = 4
#            twist.angular = -2.0
#
#            if visualize:
#                if target_angle != None:
#                    self.draw_line(this_robot, rscan, target_angle, (0, 255, 0))
#        else:
#            # Turn left
#            twist.linear = 4
#            twist.angular =  2.0
#
#            if visualize:
#                self.draw_line(this_robot, rscan, target_angle, (255, 0, 255))
#
#        return twist

    def draw_line(self, robot, rscan, angle, color):
        # Arbitrarily set for now
        range_value = 10000

        c = cos(robot.body.angle + angle)
        s = sin(robot.body.angle + angle)
        x1 = int(robot.body.position.x + rscan.INNER_RADIUS * c)
        y1 = int(robot.body.position.y + rscan.INNER_RADIUS * s)
        x2 = int(robot.body.position.x + (rscan.INNER_RADIUS + range_value) * c)
        y2 = int(robot.body.position.y + (rscan.INNER_RADIUS + range_value) * s)

        pyglet.gl.glLineWidth(6)
        pyglet.graphics.draw(2, pyglet.gl.GL_LINES, ('v2f', (x1, y1, x2, y2)),
                                                    ('c3B', color+color))
        pyglet.gl.glLineWidth(1)
