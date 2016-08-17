import rvo2, pyglet
from common import Twist, M_TO_PIXELS, MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED
from math import cos, sin, sqrt, pi, atan2

"""
A reactive collision avoidance strategy which makes use of the RVO2 library.

Important: All units in pixels!
"""

class RVOAvoider:

    NUMBER_PREF_VELS = 11
    ANGLE_MIN = -pi/2.0
    ANGLE_MAX = pi/2.0

    def __init__(self):

        # Angles of preferred velocities that will be tested each iteration.
        angles = []
        angle_delta = (self.ANGLE_MAX - self.ANGLE_MIN) / \
                      (self.NUMBER_PREF_VELS - 1)
        for i in range(self.NUMBER_PREF_VELS):
            angles.append(self.ANGLE_MIN + i * angle_delta)

        self.pref_vels = []
        for angle in angles:
            self.pref_vels.append((MAX_LINEAR_SPEED * cos(angle), \
                                   MAX_LINEAR_SPEED * sin(angle)))

        self.last_index = angles.index(0)
        self.last_mag = float('inf')

    def visualize_vel(self, robot, vx, vy, pink=False):
        x1 = (robot.body.position.x)
        y1 = (robot.body.position.y)
        world_angle = robot.body.angle + atan2(vy, vx)
        mag = sqrt(vx*vx + vy*vy)
        x2 = int(robot.body.position.x + mag * cos(world_angle))
        y2 = int(robot.body.position.y + mag * sin(world_angle))
        if pink:
            pyglet.gl.glLineWidth(5)
            pyglet.graphics.draw(2, pyglet.gl.GL_LINES,
                         ('v2f', (x1, y1, x2, y2)),
                         ('c3B', (255, 0, 255, 255, 0, 255)))
        else:
            pyglet.gl.glLineWidth(3)
            pyglet.graphics.draw(2, pyglet.gl.GL_LINES,
                         ('v2f', (x1, y1, x2, y2)),
                         ('c3B', (255, 255, 255, 255, 255, 255)))

        pyglet.gl.glLineWidth(1)

    def process_scan(self, robot, scan, visualize=False):
        # We seem to have to create a new simulator object each time because
        # otherwise it would contain the obstacles from the last time step.
        # If there was a 'removeObstacle' method it would be a bit nicer.
        sim = rvo2.PyRVOSimulator(1/60., # Time step
                                  1.5,   # neighborDist
                                  5,     # maxNeighbors
                                  1.5,   # timeHorizon (other agents)
                                  2.0,   #2     # timeHorizon (obstacles)
                                  robot.radius,   # agent radius
                                  MAX_LINEAR_SPEED)     # agent max speed
        agent = sim.addAgent((0, 0))

        # Add scan points as obstacles for the RVO simulator
        n = len(scan.ranges)
        points = []
        for i in range(0, n):
            rho = scan.INNER_RADIUS + scan.ranges[i]
            #if not (rho == float('inf') or isnan(rho)):
            theta = scan.angles[i]
            points.append((rho*cos(theta), rho*sin(theta)))

        # Obstacles as a single "negative" obstacle, with vertices specified in
        # CW order.  This requires the list of points in reverse.
        points.reverse()
        sim.addObstacle(points)
        sim.processObstacles()

        # Get the velocity in the robot reference frame with the clockwise
        # rotation matrix
        cos_theta = cos(robot.body.angle)
        sin_theta = sin(robot.body.angle)
        cur_vx = robot.body.velocity.x * cos_theta + \
                 robot.body.velocity.y * sin_theta
        cur_vy = -robot.body.velocity.x * sin_theta + \
                  robot.body.velocity.y * cos_theta

        # To prevent oscillation we will generally just test the preferred
        # velocities in the immediate neighbourhood (within the pref_vels list)
        # of the preferred velocity chosen last time.
        if self.last_mag < 20:
            # Last time the magnitude of the chosen velocity was very low.
            # Do a full search over the preferred velocities.
            start_index = 0
            stop_index = self.NUMBER_PREF_VELS - 1
        elif self.last_index == 0:
            start_index = 0
            stop_index = 1
        elif self.last_index == len(self.pref_vels)-1:
            start_index = self.NUMBER_PREF_VELS - 2
            stop_index = self.NUMBER_PREF_VELS - 1
        else:
            # This is the general case.
            start_index = self.last_index - 1
            stop_index = self.last_index + 1

        highest_mag = 0
        chosen_vel = None
        chosen_index = None
        for i in range(start_index, stop_index+1):
            pref_vel = self.pref_vels[i]

            # Initializing from scratch each time
            sim.setAgentPosition(agent, (0, 0))
            sim.setAgentVelocity(agent, (cur_vx, cur_vy))
            sim.setAgentPrefVelocity(agent, pref_vel)
            
            sim.doStep()
            (vx, vy) = sim.getAgentVelocity(0)
            #print "vel: {}, {}".format(vx, vy)
            if visualize:
                self.visualize_vel(robot, vx, vy)

            mag = sqrt(vx*vx + vy*vy)
            if mag > highest_mag:
                highest_mag = mag
                chosen_vel = (vx, vy)
                chosen_index = i

        self.last_index = chosen_index
        self.last_mag = highest_mag
        #print "highest_mag: {}".format(highest_mag)

        #chosen_vel = (avg_vx / len(self.pref_vels),
        #              avg_vy / len(self.pref_vels))

        if visualize:
            self.visualize_vel(robot, chosen_vel[0], chosen_vel[1], True)

        #print "chosen_vel: {}, {}".format(chosen_vel[0], chosen_vel[1])

        # Now treat (vx, vy) as the goal and apply the simple control law
        twist = Twist()
        twist.linear = 0.1 * chosen_vel[0]
        twist.angular = 0.02 * chosen_vel[1]
        return twist
