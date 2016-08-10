from pymunk import Body, Circle, moment_for_circle
from ..common import Pose, Twist, M_TO_PIXELS
from ..common.math import normalize_angle, unit_direction_vector


class DiffDriveBot(object):
    def __init__(self):
        self.mass = 1  # 1 kg
        # 0.1 meter radius, converted to pixels for display
        self.radius = 0.1 * M_TO_PIXELS
        # moment of inertia for disk
        rob_I = moment_for_circle(self.mass, 0, self.radius)

        self.body = Body(self.mass, rob_I)
        self.body.position = 0, 0
        self.body.angle = 0
        self.body.velocity = 0, 0
        self.body.angular_velocity = 0

        self.shape = Circle(self.body, self.radius)
        self.shape.color = 127, 0, 255  # a pinkish blue

        self._command = Twist()

    def control_step(self, dt):
        """Execute one control step for robot.

        control_step should be called regularly and at high frequency
        to ensure propper execution.

        :param dt: time since last control step execution
        :type dt: float
        """

        # convert robot body-frame input into
        # world-frame velocities for pymunk
        speed = self._command.linear.x
        #velocity = unit_direction_vector(self.body.angle) * speed

        #self.body.velocity.x = velocity.x
        #self.body.velocity.y = velocity.y
        self.body.angular_velocity = self._command.angular

        self.body.apply_impulse_at_local_point((speed, 0), (0,0))

    def set_command(self, twist):
        """Set robot velocity command.

        :param twist: command to send
        :type twist: roboticsintro.common.Twist
        """
        if twist is None:
            raise ValueError("Command may not be null. Set zero velocities instead.")
        if twist.linear.y != 0.:
            raise ValueError("DiffDriveBot cannot move sideways. twist.linear.y must be 0.")
        self._command = twist

    def stop(self):
        """Stop robot motion."""
        self.set_command(Twist())

    def get_pose(self):
        return Pose(x=self.body.position.x,
                    y=self.body.position.y,
                    theta=normalize_angle(self.body.angle))
