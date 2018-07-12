""" 
"""

from math import cos, degrees, fabs, pi, sin
from numpy import sign
from controller import Controller
from random import random
from common import Twist, ANY_LANDMARK_MASK
from common.angles import normalize_angle_pm_pi, get_angular_difference, get_smallest_angular_difference
from configsingleton import ConfigSingleton

class PushoutController(Controller):

    def __init__(self, acceptable_puck_mask):
        self.acceptable_puck_mask = acceptable_puck_mask

        config = ConfigSingleton.get_instance()

        # These properties are shared with GauciController
        self.linear_speed = config.getfloat("GauciController", "linear_speed")

        # This property is shared with LeftmostController
        self.circle_radius = config.getfloat("PushoutController", "circle_radius")

        # This property is for this controller
        self.homing_timeout = config.getfloat("PushoutController", "homing_timeout")
        self.angular_speed = config.getfloat("PushoutController", "angular_speed")

        # Possible states are PUSHING and HOMING
        self.state = "PUSHING"

    def react(self, this_robot, sensor_suite, visualize=False):
        twist = Twist()

        rscan = sensor_suite.range_scan

        # Based on the closest landmark, determine whether we are inside or
        # outside the circle
        inside = False
        lscan = sensor_suite.landmark_scan
        closest_lmark_distance = float('inf')
        closest_lmark_angle = None
        for i in range(len(lscan.ranges)):
            if (lscan.masks[i] & ANY_LANDMARK_MASK != 0
               and lscan.ranges[i] < closest_lmark_distance):
                closest_lmark_distance = lscan.ranges[i]
                closest_lmark_angle = lscan.angles[i]
        if (closest_lmark_distance == float('inf') or  # No landmarks
           closest_lmark_distance < self.circle_radius):
            inside = True

        # We use a state machine to provide some hysterisis.  If we transition
        # from inside to outside, we go into the HOMING state for a minimum
        # period.  However, if we go from 
        if self.state == "PUSHING":
            if not inside:
                self.state = "HOMING"
                self.homing_countdown = self.homing_timeout
        else: # In "HOMING"
            self.homing_countdown -= 1
            if self.homing_countdown == 0:
                self.state = "PUSHING"

        if self.state == "PUSHING":
            # Push out pucks by moving towards the weighted centroid of all
            # visible pucks.  Nearby pucks get more weight and will be targeted.
            cx = 0
            cy = 0
            pucks_in_view = False
            n = len(rscan.ranges)
            for i in range(n):
                if rscan.masks[i] & self.acceptable_puck_mask != 0:
                    angle = self.index_to_angle(rscan, i)
                    if angle >= pi/2 or angle < -pi/2:
                        continue
                    value = 1.0/(1.0 + rscan.ranges[i])
                    #value = 1.0/rscan.ranges[i]
                    #value = 1.0
                    cx = cx + value * cos(angle)
                    cy = cy + value * sin(angle)
                    pucks_in_view = True
            cx /= n
            cy /= n

            if pucks_in_view:
                twist.linear = self.linear_speed
                twist.angular = 1000.0 * cy
            else:
                # We'll move randomly if no pucks are in view.
                twist.linear = self.linear_speed
                twist.angular = 5 * (random() - 0.5)

            self.draw_line(this_robot, lscan, 0, (0, 255, 0))
        else:

            # Home towards the landmark.

            if (closest_lmark_angle <= 0):
                # Turn right
                twist.linear = self.linear_speed
                twist.angular = - self.angular_speed

                if visualize:
                    if closest_lmark_angle != None:
                        self.draw_line(this_robot, rscan, closest_lmark_angle, (0, 255, 0))
            else:
                # Turn left
                twist.linear = self.linear_speed
                twist.angular = self.angular_speed

                if visualize:
                    self.draw_line(this_robot, rscan, closest_lmark_angle, (255, 0, 255))

        self.last_inside = inside

        return twist
