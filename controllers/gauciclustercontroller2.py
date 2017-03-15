""" This version relies on having the angular range of the puck and robot
scanner's limited. """

from controller import Controller
from math import fabs, pi
from common import Twist, WALL_MASK, ROBOT_MASK, M_TO_PIXELS

class GauciClusterController2(Controller):

    def __init__(self, puck_mask):
        self.puck_mask = puck_mask

    def react(self, this_robot, sensor_suite, visualize=False):
        twist = Twist()

        scan = sensor_suite.range_scan

        # We assume the scan points straight ahead and has a single ray.
        assert len(scan.ranges) == 1
        assert scan.angles[0] == 0

        # Set the two predicates 'react_to_puck' and 'react_to_robot'.  Only
        # one should be true.
        react_to_puck = (scan.masks[0] & self.puck_mask) != 0
        react_to_robot = scan.masks[0] == ROBOT_MASK
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
                
        return twist
