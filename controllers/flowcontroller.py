""" Landmarks define flow fields which the robots move tangentially to.  Depending upon the flow field, nearby pucks are nudged towards or away from the nearest landmark. """

from controller import Controller
from math import cos, sin, pi
from random import random, normalvariate
from common import *
from common.drawing import draw_line
from configsingleton import ConfigSingleton

class FlowController(Controller):

    def __init__(self, this_robot, puck_mask):
        """
        puck_mask -- The mask for pucks this controller recognizes
        """
        self.puck_mask = puck_mask

        config = ConfigSingleton.get_instance()

        self.linear_speed = config.getfloat("FlowController", "linear_speed")
        self.angular_speed = config.getfloat("FlowController", "angular_speed")
        self.slow_factor = config.getfloat("FlowController", "slow_factor")
        self.inside_radius = config.getfloat("FlowController", "inside_radius")
        self.outside_radius = config.getfloat("FlowController",
                                             "outside_radius")
        self.wander_prob = config.getfloat("FlowController", "wander_prob")
        self.wander_mean_duration = config.getfloat("FlowController",
                                           "wander_mean_duration")
        self.wander_sigma_duration = config.getfloat("FlowController",
                                           "wander_sigma_duration")
        self.poke_prob = config.getfloat("FlowController", "poke_prob")
        self.poke_mean_duration = config.getfloat("FlowController",
                                           "poke_mean_duration")
        self.poke_sigma_duration = config.getfloat("FlowController",
                                           "poke_sigma_duration")

        self.transition_to_FLOW(this_robot)

    def transition_to_FLOW(self, this_robot):
        self.state = "FLOW"
        this_robot.shape.color = 0, 255, 0

    def transition_to_WANDER(self, this_robot):
        self.state = "WANDER"
        self.instate_count = normalvariate(self.wander_mean_duration, 
                                           self.wander_sigma_duration)
        this_robot.shape.color = 0, 0, 255

    def transition_to_POKE(self, this_robot):
        self.state = "POKE"
        self.instate_count = normalvariate(self.poke_mean_duration, 
                                           self.poke_sigma_duration)
        this_robot.shape.color = 255, 0, 0

    def get_lmark_dist_angle_mask(self, sensor_suite):
        """ Return the distance and angle of the closest landmark. """
    
        lscan = sensor_suite.landmark_scan
        closest_lmark_distance = float('inf')
        closest_lmark_angle = None
        closest_lmark_mask = None
        for i in range(len(lscan.ranges)):
            if (lscan.masks[i] & ANY_LANDMARK_MASK != 0
               and lscan.ranges[i] < closest_lmark_distance):
                closest_lmark_distance = lscan.ranges[i]
                closest_lmark_angle = lscan.angles[i]
                closest_lmark_mask = lscan.masks[i]
        if closest_lmark_angle == None:
            return None, None, None
        
        return closest_lmark_distance, closest_lmark_angle, closest_lmark_mask

    def get_leftmost_puck_index(self, rscan):
        # Go through the scan from left to right (descending order of index)
        # and choose the leftmost pixel.
        target = None
        n = len(rscan.ranges)
        for i in range(n-1, -1, -1):
            if rscan.masks[i] & self.puck_mask != 0:
                target = i
                break
        return target

    def react(self, this_robot, sensor_suite, visualize=False):

        pscan = sensor_suite.range_scan

        leftmost_puck_index = self.get_leftmost_puck_index(pscan)
        leftmost_puck_angle = None
        if leftmost_puck_index != None:
            leftmost_puck_angle = self.index_to_angle(pscan, leftmost_puck_index)

        lmark_distance, lmark_angle, lmark_mask = \
                                    self.get_lmark_dist_angle_mask(sensor_suite)

        # Check if the leftmost puck is at an angle such that we could nudge it
        # while staying aligned to the flow field.
        pokable = False
        if leftmost_puck_angle != None and lmark_distance != None:
            dot_product = (cos(leftmost_puck_angle) * cos(lmark_angle + pi/2) +
                           sin(leftmost_puck_angle) * sin(lmark_angle + pi/2))
            if dot_product > 0:
                pokable = True
                draw_line(this_robot, sensor_suite.range_scan, lmark_angle, (255, 255, 255))
                draw_line(this_robot, sensor_suite.range_scan, leftmost_puck_angle, (255, 0, 0))

        # Handle state transitions
        if self.state == "FLOW":
            if lmark_distance == None or random() < self.wander_prob:
                self.transition_to_WANDER(this_robot)
            if pokable and random() < self.poke_prob:
                self.transition_to_POKE(this_robot)
        elif self.state == "WANDER":
            self.instate_count -= 1
            if self.instate_count <= 0:
                self.transition_to_FLOW(this_robot)
        elif self.state == "POKE":
            self.instate_count -= 1
            if self.instate_count <= 0:
                self.transition_to_FLOW(this_robot)
            if not pokable:
                self.transition_to_FLOW(this_robot)
            if lmark_distance == None:
                self.transition_to_WANDER(this_robot)

        # Handle state-specific behaviour
        if self.state == "FLOW":
            return self.flow_react(this_robot, sensor_suite,
                                   lmark_angle, visualize)
        elif self.state == "WANDER":
            return self.wander_react(this_robot, sensor_suite, visualize)
        elif self.state == "POKE":
            return self.poke_react(this_robot, sensor_suite, 
                                   leftmost_puck_angle, visualize)


    def flow_react(self, this_robot, sensor_suite, lmark_angle, 
                   visualize=False):

        twist = Twist()
        if lmark_angle < -pi/2:
            # Turn right
            twist.linear = self.linear_speed
            twist.angular = - self.angular_speed

            if visualize:
                draw_line(this_robot, sensor_suite.landmark_scan, lmark_angle, (0, 255, 0))
        else:
            # Turn left
            twist.linear = self.linear_speed
            twist.angular = self.angular_speed

            if visualize:
                draw_line(this_robot, sensor_suite.landmark_scan, lmark_angle, (255, 0, 255))

        return twist

    def wander_react(self, this_robot, sensor_suite, visualize=False):
        twist = Twist()
        twist.linear = self.linear_speed
        twist.angular = 5 * (random() - 0.5)
        return twist

    def poke_react(self, this_robot, sensor_suite, leftmost_puck_angle, visualize=False):

        twist = Twist()
        if leftmost_puck_angle < 0:
            # Turn right
            twist.linear = self.linear_speed
            twist.angular = - self.angular_speed

            if visualize:
                draw_line(this_robot, sensor_suite.range_scan, leftmost_puck_angle, (0, 255, 0))
        else:
            # Turn left
            twist.linear = self.linear_speed
            twist.angular = self.angular_speed

            if visualize:
                draw_line(this_robot, sensor_suite.range_scan, leftmost_puck_angle, (255, 0, 255))

        return twist
