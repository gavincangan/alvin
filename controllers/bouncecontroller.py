""" 
"""

from math import cos, degrees, fabs, pi, sin
from numpy import sign
from controller import Controller
from random import random
from common import Twist
from common.angles import normalize_angle_pm_pi, get_angular_difference, get_smallest_angular_difference
from configsingleton import ConfigSingleton

class BounceController(Controller):

    def __init__(self, landmark_mask, acceptable_puck_mask):
        self.landmark_mask = landmark_mask
        self.acceptable_puck_mask = acceptable_puck_mask

        config = ConfigSingleton.get_instance()

        self.threshold = config.getfloat("BounceController", "threshold")

    def get_landmark_span_angles(self, lscan):
        """ Each landmark may span some angle.  Here we determine the pair of
angles for each landmark.  Each pair consists of (start_angle, stop_angle),
where the start_angle is the angle where the landmark begins as determined by
sweeping through the landmark counter-clockwise from directly behind the robot
(-pi). """

        landmark_span_angles = []

        n = len(lscan.ranges)
        # Are we starting within a landmark?
        lmark_at_0 = (lscan.masks[0] & self.landmark_mask != 0)
        within = lmark_at_0

        #print "\n\n"

        if within:
            #print "STARTING WITHIN"

            # Special case of starting within a landmark.  Search backwards
            # from the end of the scan to find where the landmark begins.
            for i in range(n-1, -1, -1):
                lmark = (lscan.masks[i] & self.landmark_mask != 0)
                if not lmark:
                    start_angle = self.index_to_angle(lscan, (i + 1) % n)
                    break

        for i in range(1, n):
            lmark = (lscan.masks[i] & self.landmark_mask != 0)
            if not within and lmark:
                # This is the start of a landmark
                start_angle = self.index_to_angle(lscan, i)
                within = True
            if within and not lmark:
                stop_angle = self.index_to_angle(lscan, i - 1)
                within = False
                landmark_span_angles.append((start_angle, stop_angle))
                #print (degrees(start_angle), degrees(stop_angle))
                
        if within and not lmark_at_0:
            # Special case of a landmark that ends at n - 1
            stop_angle = self.index_to_angle(lscan, n - 1)
            landmark_span_angles.append((start_angle, stop_angle))
            #print (degrees(start_angle), degrees(stop_angle))

        return landmark_span_angles
                
    def get_sector_widths(self, span_angles):
        """ Sector width are the angles between adjacent landmarks, defined
(for now) as the difference between the starting angle of a landmark and the
stopping angle of the previous landmark. """

        sector_widths = []

        assert len(span_angles) != 0

        n = len(span_angles)
        last_stop_angle = span_angles[0][1]
        for i in range(1, n):
            start_angle = span_angles[i][0]
            sector_width = get_angular_difference(start_angle, last_stop_angle)
            sector_widths.append(sector_width)
            #print "Sector (in degrees): " + str(degrees(sector_width))
            last_stop_angle = span_angles[i][1]

        start_angle = span_angles[0][0]
        sector_width = get_angular_difference(start_angle, last_stop_angle)
        sector_widths.append(sector_width)
        #print "LAST Sector (in degrees): " + str(degrees(sector_width))

        return sector_widths

    def careful_average_angles(self, (a, b)):
        return a + get_smallest_angular_difference(b, a) / 2.0

    def react(self, this_robot, sensor_suite, visualize=False):
        twist = Twist()

        rscan = sensor_suite.range_scan
        lscan = sensor_suite.landmark_scan

        span_angles = self.get_landmark_span_angles(lscan)
        sector_widths = self.get_sector_widths(span_angles)

        assert len(span_angles) == len(sector_widths)

        # If any sector width is greater than the threshold then we will turn
        # away from that sector.  Otherwise, we will continue forwards.  If
        # there is more than one such "bad sector" then we choose the one with
        # the smallest width.
        n = len(sector_widths)
        index_of_bad_sector = None
        smallest_bad_width = float('inf')
        for i in range(0, n):
            if (sector_widths[i] > self.threshold
               and sector_widths[i] < smallest_bad_width):
                index_of_bad_sector = i
                smallest_bad_width = sector_widths[i]

        #print index_of_bad_sector
        if index_of_bad_sector != None:
            lmark_a_angle = self.careful_average_angles(span_angles[(index_of_bad_sector + 1) % n])
            lmark_b_angle = self.careful_average_angles(span_angles[index_of_bad_sector])
            #print "LANDMARK ANGLES (in degrees)"
            #print "A (yellow): " + str(degrees(lmark_a_angle))
            #print "B (white): " + str(degrees(lmark_b_angle))

            # Choose direction to react based on the average of the
            # corresponding landmark angles.
            react_angle = (lmark_a_angle +
                get_smallest_angular_difference(lmark_b_angle,
                                                lmark_a_angle) / 2.0)
            
            twist.linear = 10.0
            twist.angular = 5 * sign(normalize_angle_pm_pi(react_angle))

            self.draw_line(this_robot, lscan, lmark_a_angle, (255, 255, 0))
            self.draw_line(this_robot, lscan, lmark_b_angle, (255, 255, 255))
            self.draw_line(this_robot, lscan, react_angle, (255, 0, 0))
        else:
            # Compute centroid of all pucks in the robot's ref. frame
            cx = 0
            cy = 0
            n = len(rscan.ranges)
            for i in range(n):
                if rscan.masks[i] & self.acceptable_puck_mask != 0:
                    angle = self.index_to_angle(rscan, i)
                    if angle >= pi/2 or angle < -pi/2:
                        continue
                    #value = 1.0/rscan.ranges[i]
                    #value = 1.0/rscan.ranges[i]
                    value = 1.0
                    cx = cx + value * cos(angle)
                    cy = cy + value * sin(angle)
            cx /= n
            cy /= n

            twist.linear = 10.0
            twist.angular = 100.0 * cy


            self.draw_line(this_robot, lscan, 0, (0, 255, 0))

        return twist
