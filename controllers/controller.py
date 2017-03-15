""" An abstract class giving the form all concrete controllers should have. """
class Controller:

    def react(self, robot, sensor_suite, visualize=False):
        """ Given the robot and it's sensor suite (a dictionary) determine
            how the robot should react by returning a Twist. """
        raise NotImplementedError()
