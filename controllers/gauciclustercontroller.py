from controller import Controller
from math import fabs, pi
from common import Twist, M_TO_PIXELS

FRONT_THRESHOLD = 10.0 * pi / 180.0

class GauciClusterController(Controller):

    def react(self, this_robot, sensor_suite, visualize=False):
        twist = Twist()

        pucks = sensor_suite.puck_scan.pucks
        robots = sensor_suite.robot_scan.robots

        # First find the puck with the smallest absolute angle.
        frontal_puck_abs_angle = float('inf')
        frontal_puck_range = float('inf')
        for puck in pucks:
            if puck.kind == 0 and fabs(puck.angle) < frontal_puck_abs_angle:
                frontal_puck_abs_angle = fabs(puck.angle)
                frontal_puck_range = puck.distance

        # Now find the robot with the smallest absolute angle.
        frontal_bot_abs_angle = float('inf')
        frontal_bot_range = float('inf')
        for robot in robots:
            if fabs(robot.angle) < frontal_bot_abs_angle:
                frontal_bot_abs_angle = fabs(robot.angle)
                frontal_bot_range = robot.distance

        # Set the two predicates 'react_to_puck' and 'react_to_robot'
        react_to_puck = frontal_puck_abs_angle <= FRONT_THRESHOLD
        react_to_robot = frontal_bot_abs_angle <= FRONT_THRESHOLD
        if react_to_puck and react_to_robot:
            # Choose which is closer, then turn off the other.
            if frontal_puck_range < frontal_bot_range:
                react_to_robot = False
            else:
                react_to_puck = False

        # At this point only one of the two predicates should be true.
        assert not (react_to_robot and react_to_puck)

        # Now react...
        if react_to_puck:
            # Turn right
            twist.linear = 4
            twist.angular = -2.0
        elif react_to_robot:
            # Turn left and slow
            twist.linear = 2
            twist.angular = 2.0
        else:
            # Turn left
            twist.linear = 4
            twist.angular = 2.0
                
        return twist
