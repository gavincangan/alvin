from pymunk import Body, Circle, moment_for_circle, ShapeFilter
from common import Twist, RED_PUCK_MASK, GREEN_PUCK_MASK, BLUE_PUCK_MASK, M_TO_PIXELS

class Puck(object):
    def __init__(self, kind):
        self.mass = 0.1  # 0.1 kg
        # 0.05 meter radius, converted to pixels for display
        self.radius = 0.10 * M_TO_PIXELS
        # moment of inertia for disk
        moment_inertia = moment_for_circle(self.mass, 0, self.radius)

        self.body = Body(self.mass, moment_inertia)
        self.body.position = 0, 0
        self.body.angle = 0
        self.body.velocity = 0, 0
        self.body.angular_velocity = 0

        self.shape = Circle(self.body, self.radius)

        self.kind = kind
        if kind == 0:
            self.shape.color = 255, 0, 0
            self.shape.filter = ShapeFilter(categories = RED_PUCK_MASK)
        elif kind == 1:
            self.shape.color = 0, 255, 0
            self.shape.filter = ShapeFilter(categories = GREEN_PUCK_MASK)
        elif kind == 2:
            self.shape.color = 0, 0, 255
            self.shape.filter = ShapeFilter(categories = BLUE_PUCK_MASK)
        else:
            sys.exit("Unknown puck kind: " + kind)
