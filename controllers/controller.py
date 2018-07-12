from math import sin, cos

""" An abstract class giving the form all concrete controllers should have. """
class Controller:

    def react(self, robot, sensor_suite, visualize=False):
        """ Given the robot and it's sensor suite (a dictionary) determine
            how the robot should react by returning a Twist. """
        raise NotImplementedError()

    def index_to_angle(self, scan, index):
        if index == None:
            return None
        else:
            return (scan.ANGLE_MIN + index * 
                (scan.ANGLE_MAX - scan.ANGLE_MIN) / float(scan.NUMBER_POINTS))
