#!/usr/bin/env python

import pyglet
import pymunk
import pymunk.pyglet_util
import copy

from math import pi, cos, sin, sqrt
from pyglet.window import key
from pymunk import Vec2d, ShapeFilter
from random import seed, randint, choice

from puck import Puck
from robot import Robot
from common import Twist, WALL_MASK, ROBOT_MASK, RED_PUCK_MASK, GREEN_PUCK_MASK, BLUE_PUCK_MASK, ANY_PUCK_MASK, M_TO_PIXELS, \
                   MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED
from sensors import RangeScan, RangeScanner, PuckScan, PuckScanner, RobotScan, \
                    RobotScanner
from sensorsuite import SensorSuite
from controllers import *


# Parameters passed on to controller
#SIM_STEPS = 1

class AlvinSim(pyglet.window.Window):

    #
    # Parameters
    #
    WIDTH = 1000     # Dimensions of the arena (in pixels).
    HEIGHT = 1000
    N_ROBOTS = 10
    N_PUCKS = 10
    PUCK_KINDS = [0] # List of types of pucks to choose from

    # UI related
    STEPS_BETWEEN_TOGGLES = 20
    visualize_sensors = False
    visualize_controllers = True
    allow_control = True
    steps_since_toggle = 0

    # Analysis related
    steps = 0
    collisions = 0
    cum_speed = 0

    def __init__(self):
        super(AlvinSim, self).__init__(self.WIDTH, self.HEIGHT, visible=False)

        # Setup visualization and user input
        #self.window = pyglet.window.Window(self.width, self.height, \
        #                                   visible=False)

        self.keyboard = key.KeyStateHandler()
        self.push_handlers(self.keyboard)
        self.draw_options = pymunk.pyglet_util.DrawOptions()
        self.draw_options.flags = self.draw_options.DRAW_SHAPES

        # Help text
        self.helpLabel = pyglet.text.Label(
            'ESC: quit, arrow-keys: move, s: vis sens, c: vis ctrls, a: auton',
            font_size=12,
            x=self.width//2, y=self.height//20,
            anchor_x='center', anchor_y='center')

        # Label to show various stats
        self.statsLabel = pyglet.text.Label(
            font_size=18,
            x=self.width//2, y=self.height - 30,
            anchor_x='center', anchor_y='center')

        self.set_stats_label_text()

        # Objects for mouse interaction
        self.selected = None
        self.mouse_body = pymunk.Body(body_type = pymunk.Body.KINEMATIC)

        # build simulation environment
        self.env = pymunk.Space()
        self.env.damping = 0.01 # 99% of velocity is lost per second

        # Walls of the environment.
        wall_thickness = 10
        env_b = self.env.static_body
        walls = [
            pymunk.Segment(env_b, Vec2d(0, 0), Vec2d(self.WIDTH, 0), \
                           wall_thickness),
            pymunk.Segment(env_b, Vec2d(self.WIDTH, 0), Vec2d(self.WIDTH, self.HEIGHT), \
                           wall_thickness),
            pymunk.Segment(env_b, Vec2d(self.WIDTH, self.HEIGHT), Vec2d(0, self.HEIGHT), \
                           wall_thickness),
            pymunk.Segment(env_b, Vec2d(0, self.HEIGHT), Vec2d(0,0), \
                           wall_thickness)
        ]
        for wall_shape in walls:
            wall_shape.filter = ShapeFilter(categories = WALL_MASK)
        self.env.add(walls)

        # Seed random number generator.
        #seed(1)

        # A few randomly distributed walls.
        random_walls = []
        #n = randint(10, 20)
        n = 0 # Open environment
        for i in range(n):
            x1, y1 = randint(0, self.WIDTH), randint(0, self.HEIGHT)
            angle = pi/2. * randint(0,3)
            length = randint(wall_thickness, self.WIDTH/2)
            x2, y2 = x1 + length*cos(angle), y1 + length*sin(angle)
            wall_shape = pymunk.Segment(env_b, Vec2d(x1,y1), Vec2d(x2,y2), \
                                        wall_thickness)
            wall_shape.filter = ShapeFilter(categories = WALL_MASK)
            random_walls.append(wall_shape)
        self.env.add(random_walls)
        
        # Create the robots
        self.robots = []
        for i in range(self.N_ROBOTS):
            # We vary the mask used to detect pucks for both the range sensor
            # and the controller.
            puck_mask = ANY_PUCK_MASK
            #puck_mask = None
            #if i % 3 == 0:
            #    puck_mask = RED_PUCK_MASK
            #elif i % 3 == 1:
            #    puck_mask = GREEN_PUCK_MASK
            #elif i % 3 == 2:
            #    puck_mask = BLUE_PUCK_MASK

            robot = Robot()
            offset = wall_thickness + robot.radius
            placed = False
            while not placed:
                x = randint(offset, self.WIDTH - offset)
                y = randint(offset, self.HEIGHT - offset)
                robot.body.position = x, y
                if self.env.shape_query(robot.shape) == []:
                    placed = True
            self.env.add(robot.body, robot.shape)

            # Create the robot's sensors
            robot.range_scanner = RangeScanner(WALL_MASK|ROBOT_MASK|ANY_PUCK_MASK,
                                               WALL_MASK|ROBOT_MASK|puck_mask)
#            robot.puck_scanner = PuckScanner()
#            robot.robot_scanner = RobotScanner()

            # Create the controller
            #robot.controller = SimpleAvoiderController()
            #robot.controller = RVOAvoiderController(SIM_STEPS)
            #robot.controller = GauciClusterController()
            #robot.controller = GauciClusterController2(puck_mask)
            robot.controller = MyController(puck_mask)

            self.robots.append(robot)

        # Create the pucks
        self.pucks = []
        for i in range(self.N_PUCKS):
            puck = Puck(choice(self.PUCK_KINDS))
            offset = wall_thickness + puck.radius
            placed = False
            while not placed:
                x = randint(offset, self.WIDTH - offset)
                y = randint(offset, self.HEIGHT - offset)
                puck.body.position = x, y
                if self.env.shape_query(puck.shape) == []:
                    placed = True
            self.env.add(puck.body, puck.shape)
            self.pucks.append(puck)

        # Schedule the key callbacks
        for robot in self.robots:
            pyglet.clock.schedule_interval(robot.control_step, 1.0/120)
        pyglet.clock.schedule_interval(self.update, 1.0/60)
        pyglet.clock.schedule_interval(self.env.step, 1.0/60)

        # Setup to handle collisions
        self.env.add_default_collision_handler().begin = self.collision_handler

        # start simulation
        self.set_visible(True)
        pyglet.app.run()

    def unschedule(self):
        for robot in self.robots:
            pyglet.clock.unschedule(robot.control_step)
        pyglet.clock.unschedule(self.update)
        pyglet.clock.unschedule(self.env.step)

    def set_stats_label_text(self):
        self.statsLabel.text = \
            "steps: {}, collisions: {}, cum. speed: {:.0f}".format(self.steps, self.collisions, self.cum_speed)

    def collision_handler(self, arbiter, space, data):
        self.collisions += 1
        return True

    # define how to draw the visualization
    def on_draw(self):
        # always clear and redraw for graphics programming
        self.clear()
        self.env.debug_draw(self.draw_options)
        self.helpLabel.draw()
        self.set_stats_label_text()
        self.statsLabel.draw()
        if self.visualize_sensors or self.visualize_controllers:
            for robot in self.robots:
#                range_scan = None
                range_scan = robot.range_scanner.compute(self.env, robot, \
                                                         self.visualize_sensors)
#                puck_scan = robot.puck_scanner.compute(self.env, robot, \
#                                                       self.pucks, True)
#                robot_scan = robot.robot_scanner.compute(self.env, robot, \
#                                                         self.robots, True)
        

#                sensor_suite = SensorSuite(range_scan, puck_scan, robot_scan)
                sensor_suite = SensorSuite(range_scan)
                # Call the controller's react method, although we will actually
                # ignore the resulting twist here.
                robot.controller.react(robot, sensor_suite, \
                                       self.visualize_controllers)

        #if self.steps % 10 == 0:
        #   file_name = '{:09d}.png'.format(self.steps)
        #   pyglet.image.get_buffer_manager().get_color_buffer().save(file_name)

    def on_mouse_press(self, x, y, button, modifiers):
        self.mouse_body.position = x,y
        hit = self.env.point_query_nearest((x,y), 10, pymunk.ShapeFilter())
        if hit != None:
            body = hit.shape.body
            if body.body_type == pymunk.Body.DYNAMIC:
                rest_length = self.mouse_body.position.get_distance(\
                                                                body.position)
                stiffness = 500
                damping = 10
                self.selected = pymunk.DampedSpring(self.mouse_body, body, \
                                  (0,0), (0,0), rest_length, stiffness, damping)
                self.env.add(self.selected)
            
    def on_mouse_release(self, x, y, button, modifiers):
        if self.selected != None:
            self.env.remove(self.selected)
            self.selected = None
        
    def on_mouse_drag(self, x, y, dx, dy, buttons, modifiers):
        self.mouse_body.position = x,y


    def handle_keys(self):
        manual_twist = Twist()

        if self.keyboard[key.RIGHT]:
            manual_twist.angular = -0.25 * MAX_ANGULAR_SPEED
        if self.keyboard[key.LEFT]:
            manual_twist.angular = 0.25 * MAX_ANGULAR_SPEED
        if self.keyboard[key.UP]:
            manual_twist.linear = 0.25 * MAX_LINEAR_SPEED
        if self.keyboard[key.DOWN]:
            manual_twist.linear = -0.25 * MAX_LINEAR_SPEED

        # Handle the keyboard events which shouldn't be toggled too quickly.
        self.steps_since_toggle += 1
        if self.keyboard[key.S] and \
            self.steps_since_toggle > self.STEPS_BETWEEN_TOGGLES:

            self.visualize_sensors = not self.visualize_sensors
            self.steps_since_toggle = 0
        if self.keyboard[key.C] and \
            self.steps_since_toggle > self.STEPS_BETWEEN_TOGGLES:

            self.visualize_controllers = not self.visualize_controllers
            self.steps_since_toggle = 0
        if self.keyboard[key.A] and \
            self.steps_since_toggle > self.STEPS_BETWEEN_TOGGLES:

            self.allow_control = not self.allow_control
            steps_since_toggle = 0

        return manual_twist

    def update(self, dt):
        manual_twist = self.handle_keys()

        for robot in self.robots:
            self.update_for_robot(dt, robot, manual_twist)
            self.cum_speed += robot.body.velocity.get_length()

        self.steps += 1
        #    #pyglet.app.exit()
        #    self.unschedule()
        #    self.close()

    def update_for_robot(self, dt, robot, manual_twist):

        # First do autonomous control
#        range_scan = None
        range_scan = robot.range_scanner.compute(self.env, robot, False)
#        puck_scan = robot.puck_scanner.compute(self.env, robot, self.pucks, \
#                                               False)
#        robot_scan = robot.robot_scanner.compute(self.env, robot, self.robots, \
#                                                 False)
        #print scan.ranges
#        sensor_suite = SensorSuite(range_scan, puck_scan, robot_scan)
        sensor_suite = SensorSuite(range_scan)
        controller_twist = robot.controller.react(robot, sensor_suite, False)
        twist = copy.deepcopy(manual_twist)
        if self.allow_control:
            # Combine manual and controller twists
            twist.linear += controller_twist.linear
            twist.angular += controller_twist.angular

        if twist.linear > MAX_LINEAR_SPEED:
            twist.linear = MAX_LINEAR_SPEED
        if twist.linear < -MAX_LINEAR_SPEED:
            twist.linear = -MAX_LINEAR_SPEED
        if twist.angular > MAX_ANGULAR_SPEED:
            twist.angular = MAX_ANGULAR_SPEED
        if twist.angular < -MAX_ANGULAR_SPEED:
            twist.angular = -MAX_ANGULAR_SPEED

        robot.set_command(twist)


# make module runnable from command line
if __name__ == '__main__':
    global SIM_STEPS

# EXPERIMENT VARING 'SIM_STEPS'
#
#    for i in range(1, 20):
#        if i == 0:
#            SIM_STEPS = 1
#        else:
#            SIM_STEPS = i
#        print "SIM_STEPS: {}".format(SIM_STEPS)
#        sim = AlvinSim()

    SIM_STEPS = 1
    sim = AlvinSim()
