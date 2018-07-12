""" A combination of strategies used to build a circular enclosure centred on a
landmark.  The absence of a visible landmark indicates an area to be cleared
such that arcs of circles can be constructed.  Partially inspired by Gauci et
al's controller from 'Clustering objects with robots that do not compute'. """

from controller import Controller
from math import fabs, pi, cos, sin
from random import random
from common import *
from common.drawing import draw_line
from common.angles import normalize_angle_pm_pi
from configsingleton import ConfigSingleton

class LandmarkCircleController(Controller):

    def __init__(self, this_robot, puck_mask):
        """
        puck_mask -- The mask for pucks this controller recognizes
        """
        self.puck_mask = puck_mask

        config = ConfigSingleton.get_instance()

        # These properties are shared with GauciController
        self.front_angle_threshold = config.getfloat("GauciController",
                                            "front_angle_threshold")
        self.linear_speed = config.getfloat("GauciController", "linear_speed")
        self.angular_speed = config.getfloat("GauciController", "angular_speed")
        self.slow_factor = config.getfloat("GauciController", "slow_factor")

        # These properties are for this controller
        self.inside_radius = config.getfloat("LandmarkCircleController",
                                             "inside_radius")
        self.outside_radius = config.getfloat("LandmarkCircleController",
                                             "outside_radius")
        self.homing_timeout = config.getfloat("LandmarkCircleController",
                                              "homing_timeout")
        self.homing_angular_speed = config.getfloat("LandmarkCircleController",
                                             "homing_angular_speed")
        self.homing_angular_speed = config.getfloat("LandmarkCircleController",
                                             "homing_angular_speed")
        self.outie_trans_prob = config.getfloat("LandmarkCircleController",
                                             "outie_trans_prob")

        """
        We will randomly assign 'outie'.  An "outie" moves out from arc
        landmarks and then acts according to a variant of Gauci et al's
        controller.  An "innie" (outie == False) pushes pucks outwards when
        within the desired radius and tries to maintain itself within that
        radius.
        """
        self.outie = (random() < 0.5)
        self.outie_changed(this_robot)

        # Initial state for innies.  Possible states are PUSHING and HOMING.
        self.innie_state = "PUSHING"

    def outie_changed(self, this_robot):
        if self.outie:
            this_robot.shape.color = 0, 255, 0
        else:
            this_robot.shape.color = 0, 0, 255

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

    def pushout_behaviour(self, this_robot, scan):
        # Push out pucks by moving towards the weighted centroid of all
        # visible pucks.  Nearby pucks get more weight and will be targeted.
        cx = 0
        cy = 0
        pucks_in_view = False
        n = len(scan.ranges)
        for i in range(n):
            if scan.masks[i] & self.puck_mask != 0:
                angle = self.index_to_angle(scan, i)
                if angle >= pi/2 or angle < -pi/2:
                    continue
                value = 1.0/(1.0 + scan.ranges[i])
                #value = 1.0/scan.ranges[i]
                #value = 1.0
                cx = cx + value * cos(angle)
                cy = cy + value * sin(angle)
                pucks_in_view = True
        cx /= n
        cy /= n

        twist = Twist()
        if pucks_in_view:
            twist.linear = self.linear_speed
            twist.angular = 1000.0 * cy
        else:
            # We'll move randomly if no pucks are in view.
            twist.linear = self.linear_speed
            twist.angular = 5 * (random() - 0.5)

        #self.draw_line(this_robot, lscan, 0, (0, 255, 0))

        return twist

    def wander_behaviour(self, this_robot, scan):
        twist = Twist()
        twist.linear = self.linear_speed
        twist.angular = 5 * (random() - 0.5)
        return twist

    def home_to_angle(self, this_robot, angle, visualize=False):
        # Home towards the landmark.
        twist = Twist()
        if (angle <= 0):
            # Turn right
            twist.linear = self.linear_speed
            twist.angular = - self.homing_angular_speed

        else:
            # Turn left
            twist.linear = self.linear_speed
            twist.angular = self.homing_angular_speed

        if visualize:
            if angle != None:
                draw_line(this_robot, scan, angle, (255, 255, 255))

        return twist

    def react(self, this_robot, sensor_suite, visualize=False):

        # Toggle outie with a small probability
        if random() < self.outie_trans_prob:
            self.outie = not self.outie
            self.outie_changed(this_robot)

        if self.outie:
            return self.outie_react(this_robot, sensor_suite, visualize)
        else:
            return self.innie_react(this_robot, sensor_suite, visualize)


    def outie_react(self, this_robot, sensor_suite, visualize=False):
        twist = Twist()

        scan = sensor_suite.range_scan
        n = len(scan.ranges)
        centre_angle = 0

        lmark_distance, lmark_angle, lmark_mask = \
                                    self.get_lmark_dist_angle_mask(sensor_suite)

        # If no landmark is in view, then move towards the
        if (lmark_mask == ARC_LANDMARK_MASK
            and lmark_distance < self.outside_radius):
            return self.home_to_angle(this_robot, 
                                normalize_angle_pm_pi(lmark_angle + pi))


        # If there is a blast landmark then clear the area.
        if lmark_distance == None or lmark_mask == BLAST_LANDMARK_MASK:
            return self.pushout_behaviour(this_robot, scan)

        # If the closest landmark is an ARC and we are within the outer radius,
        # then we will move away from it.
        if (lmark_mask == ARC_LANDMARK_MASK
            and lmark_distance < self.outside_radius):
            return self.home_to_angle(this_robot, 
                                normalize_angle_pm_pi(lmark_angle + pi))

        # If we are within the outer radius, then we will steer away from pucks
        # by altering our definition of the image centre.
#        if lmark_distance < self.outside_radius:
#            centre_angle = 0.3

        # Relying on only a single forward-pointing ray causes distant pucks
        # to be easily missed (they may be detected, but too sporadically to
        # attract the robot).  We accept as forward-pointing any sensor ray
        # within 'front_angle_threshold' of zero.  Correspondingly set the
        # two predicates 'react_to_puck' and 'react_to_robot'.
        react_to_puck = False
        react_to_robot = False

        # We will treat POLE landmarks as pucks.  See if one is centred.
        if lmark_mask == POLE_LANDMARK_MASK:
            lscan = sensor_suite.landmark_scan
            nl = len(lscan.ranges)
            for i in range(nl):
                if (fabs(lscan.angles[i] + centre_angle) <
                   2*self.front_angle_threshold):
                    if (lscan.masks[i] & POLE_LANDMARK_MASK) != 0:
                        react_to_puck = True

        for i in range(n):
            if fabs(scan.angles[i] + centre_angle) <self.front_angle_threshold:
                if (scan.masks[i] & self.puck_mask) != 0:
                    react_to_puck = True
                if scan.masks[i] == ROBOT_MASK:
                    react_to_robot = True

        if react_to_robot:
            react_to_puck = False

        # Now react...
        if react_to_puck:
            # Turn left
            twist.linear = self.linear_speed
            twist.angular =  self.angular_speed
            if visualize:
                draw_line(this_robot, scan, 0, (255, 0, 255))

        elif react_to_robot:
            # Turn left and slow
            twist.linear = self.linear_speed * self.slow_factor
            twist.angular = self.angular_speed

            if visualize: # Reacting to robot
                draw_line(this_robot, scan, 0, (255, 0, 0))
        else:
            # Turn right
            twist.linear = self.linear_speed
            twist.angular = -self.angular_speed

            if visualize:
                draw_line(this_robot, scan, 0, (255, 0, 255))
                
        return twist

    def innie_react(self, this_robot, sensor_suite, visualize=False):
        scan = sensor_suite.range_scan

        lmark_distance, lmark_angle, lmark_mask = \
                                    self.get_lmark_dist_angle_mask(sensor_suite)

        # If there is no landmark or a blast landmark then clear the area.
        if lmark_distance == None or lmark_mask == BLAST_LANDMARK_MASK:
            return self.pushout_behaviour(this_robot, scan)

        if lmark_mask == POLE_LANDMARK_MASK:
            return self.wander_behaviour(this_robot, scan)

        inside = (lmark_distance < self.inside_radius)

        # We use a state machine to provide some hysterisis.  If we transition
        # from inside to outside, we go into the HOMING state for a minimum
        # period.  However, if we go from 
        if self.innie_state == "PUSHING":
            if not inside:
                self.innie_state = "HOMING"
                self.homing_countdown = self.homing_timeout
        else: # In "HOMING"
            self.homing_countdown -= 1
            if self.homing_countdown == 0:
                self.innie_state = "PUSHING"

        if self.innie_state == "PUSHING":
            return self.pushout_behaviour(this_robot, scan)
        else:

            return self.home_to_angle(this_robot, lmark_angle)
