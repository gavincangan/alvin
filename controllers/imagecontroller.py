""" Builds and operates on a 1D wraparound image.  """

from controller import Controller
from math import fabs, pi, sin, cos
from common import Twist
from common.angles import normalize_angle_pm_pi
from configsingleton import ConfigSingleton

class ImageController(Controller):

    def __init__(self, acceptable_puck_mask):
        self.acceptable_puck_mask = acceptable_puck_mask

        config = ConfigSingleton.get_instance()

        # These properties are shared with GauciController
        self.linear_speed = config.getfloat("GauciController", "linear_speed")
        self.angular_speed = config.getfloat("GauciController", "angular_speed")
        self.slow_factor = config.getfloat("GauciController", "slow_factor")

    def react(self, this_robot, sensor_suite, visualize=False):
        twist = Twist()

        rscan = sensor_suite.range_scan
        n = len(rscan.ranges)

        target = self.get_target(rscan)

        # Convert to an angle
        if target == None:
            target_angle = None
        else:
            target_angle = (rscan.ANGLE_MIN + target * (rscan.ANGLE_MAX - rscan.ANGLE_MIN) / float(rscan.NUMBER_POINTS))

        if self.modulate and target_angle != None:
            compass_angle = normalize_angle_pm_pi(this_robot.body.angle)
            if cos(2*compass_angle) < 0:
                # Dig in to push the puck more
                target_angle = target_angle - self.ingress_angle
            else:
                # Pull out to skirt the puck
                target_angle = target_angle + self.egress_angle

        # Check for the presence of another robot in the centre of the image.
        react_to_robot = False
        react_to_robot_angle = None
        for i in range(0, n):
            if (rscan.masks[i] == ROBOT_MASK and fabs(rscan.angles[i]) < pi/4 and
                rscan.ranges[i] < 2*this_robot.radius):
                react_to_robot = True
                react_to_robot_angle = (rscan.ANGLE_MIN + i * 
                    (rscan.ANGLE_MAX - rscan.ANGLE_MIN) / float(rscan.NUMBER_POINTS))

        if react_to_robot:
            # Turn left and slow
            twist.linear = self.slow_factor * self.linear_speed
            twist.angular = self.angular_speed

            if visualize: # Reacting to robot
                self.draw_line(this_robot, rscan, react_to_robot_angle,
                               (255, 0, 0))

        elif (target_angle == None or target_angle <= 0):
            # Turn right
            twist.linear = self.linear_speed
            twist.angular = - self.angular_speed

            if visualize:
                if target_angle != None:
                    self.draw_line(this_robot, rscan, target_angle, (0, 255, 0))
        else:
            # Turn left
            twist.linear = self.linear_speed
            twist.angular = self.angular_speed

            if visualize:
                self.draw_line(this_robot, rscan, target_angle, (255, 0, 255))

        return twist
