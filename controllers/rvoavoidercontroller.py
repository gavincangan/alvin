from controller import Controller
import rvo2, pyglet
from common import Twist, M_TO_PIXELS, MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED
from math import cos, sin, sqrt, pi, atan2

"""
A reactive collision avoidance strategy which makes use of the RVO2 library.

Important: All units in pixels!
"""

class RVOAvoiderController(Controller):

    NUMBER_PREF_VELS = 11
    ANGLE_MIN = -pi/2.0
    ANGLE_MAX = pi/2.0
    SIM_STEPS = 1

    def __init__(self, sim_steps=1):

        self.SIM_STEPS = sim_steps

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

    def draw_line_from_robot(self, robot, vx, vy, red, green, blue, thickness):
        x1 = (robot.body.position.x)
        y1 = (robot.body.position.y)
        world_angle = robot.body.angle + atan2(vy, vx)
        mag = sqrt(vx*vx + vy*vy)
        x2 = int(robot.body.position.x + mag * cos(world_angle))
        y2 = int(robot.body.position.y + mag * sin(world_angle))
        pyglet.gl.glLineWidth(thickness)
        pyglet.graphics.draw(2, pyglet.gl.GL_LINES,
                     ('v2f', (x1, y1, x2, y2)),
                     ('c3B', (red, green, blue, red, green, blue)))
        pyglet.gl.glLineWidth(1)

    def react(self, robot, sensor_suite, visualize=False):

        range_scan = sensor_suite.range_scan
        #puck_scan = sensor_suite.puck_scan

        # We seem to have to create a new simulator object each time because
        # otherwise it would contain the obstacles from the last time step.
        # If there was a 'removeObstacle' method it would be a bit nicer.
        sim = rvo2.PyRVOSimulator(1/60., # Time step
                                  1.5,   # neighborDist
                                  5,     # maxNeighbors
                                  1.5,   # timeHorizon (other agents)
                                  1.5,   #2     # timeHorizon (obstacles)
                                  robot.radius,   # agent radius
                                  MAX_LINEAR_SPEED)     # agent max speed
        agent = sim.addAgent((0, 0))

        # Add range scan points as obstacles for the RVO simulator
        n = len(range_scan.ranges)
        points = []
        for i in range(0, n):
            rho = range_scan.INNER_RADIUS + range_scan.ranges[i]
            #if not (rho == float('inf') or isnan(rho)):
            theta = range_scan.angles[i]
            points.append((rho*cos(theta), rho*sin(theta)))

        # Add pucks from the puck scan
        #for puck in puck_scan.pucks:
        #    rho = puck.distance
        #    theta = puck.angle
        #    points.append((rho*cos(theta), rho*sin(theta)))

        # Add fake points behind the robot to make it think twice about going
        # backwards.
        #n_fake = 0
        #start_angle = range_scan.ANGLE_MAX
        #stop_angle = range_scan.ANGLE_MIN + 2*pi
        #angle_delta = (stop_angle - start_angle) / (n_fake - 1)
        #for i in range(n_fake):
        #    theta = start_angle + i * angle_delta
        #    rho = 2 * robot.radius
        #    points.append((rho*cos(theta), rho*sin(theta)))
        #    if visualize:
        #        vx,vy = rho*cos(theta), rho*sin(theta)
        #        self.draw_line_from_robot(robot, vx, vy, 0, 0, 255, 1)

        # The scan points will be treated together as a single "negative"
        # obstacle, with vertices specified in CW order.  This requires the
        # following sort.
        points.sort(key = lambda p: -atan2(p[1], p[0]))
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
            
            for j in range(self.SIM_STEPS):
                sim.doStep()

            (vx, vy) = sim.getAgentVelocity(0)
            #print "vel: {}, {}".format(vx, vy)
            if visualize:
                self.draw_line_from_robot(robot, vx, vy, 255, 255, 255, 3)

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

        if visualize and chosen_vel != None:
            self.draw_line_from_robot(robot, chosen_vel[0], chosen_vel[1], 255, 0, 127, 5)

        #print "MAX_LINEAR_SPEED: {}".format(MAX_LINEAR_SPEED)
        #print "current_vel: {}, {}".format(cur_vx, cur_vy)
        #print "MAG OF current_vel: {}".format(sqrt(cur_vx**2+ cur_vy**2))
        #print "chosen_vel: {}, {}".format(chosen_vel[0], chosen_vel[1])
        #print "MAG OF chosen_vel: {}".format(sqrt(chosen_vel[0]**2+ chosen_vel[1]**2))

        # Now treat (vx, vy) as the goal and apply the simple control law
        twist = Twist()
        if chosen_vel != None:
            twist.linear = 0.1 * chosen_vel[0]
            twist.angular = 0.02 * chosen_vel[1]
        else:
            print "NO AVAILABLE VELOCITY!"
            #for r in range_scan.ranges:
            #    print r
        return twist
