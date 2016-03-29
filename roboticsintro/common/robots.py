from pymunk import Body, Circle, moment_for_circle
from ..common import Pose, Twist, M_TO_PIXELS
from ..common.math import normalize_angle


class DiffDriveBot(object):
    def __init__(self):
        rob_mass = 1  # 1 kg
        # 0.1 meter radius, converted to pixels for display
        rob_radius = 0.1 * M_TO_PIXELS
        # moment of inertia for disk
        rob_I = moment_for_circle(rob_mass, 0, rob_radius)

        self.body = Body(rob_mass, rob_I)
        self.body.position = 0, 0
        self.body.angle = 0
        self.body.velocity = 0, 0
        self.body.angular_velocity = 0

        self.shape = Circle(self.body, rob_radius)
        self.shape.color = 255, 0, 0  # red

    def set_command(self, twist):
        """Set robot velocity command.

        :param twist: command to send
        :type twist: roboticsintro.common.Twist
        """
        if twist is None:
            raise ValueError("Command may not be null. Set zero velocities instead.")

        self.body.velocity = twist.linear.pair()
        self.body.angular_velocity = twist.angular

    def stop(self):
        """Immediately stop robot motion."""
        self.set_command(Twist())

    def get_pose(self):
        return Pose(x=self.body.position.x,
                    y=self.body.position.y,
                    theta=normalize_angle(self.body.angle))
