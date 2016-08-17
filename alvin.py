#!/usr/bin/env python

import pyglet
import pymunk
import pymunk.pyglet_util
import copy

from math import pi, cos, sin, sqrt
from pyglet.window import key
from pymunk import Vec2d
from random import randint

from common import Twist, M_TO_PIXELS, MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED
from robots import DiffDriveBot
from sensors import Scan, RangeScanner
from controllers import *

N_ROBOTS = 1
STEPS_BETWEEN_TOGGLES = 5

debug = True
allow_control = True
steps_since_toggle = 0
collisions = 0

def unit_direction_vector(theta=0.0):
    return cos(theta), sin(theta)

def run():
    # Dimensions of the arena (in pixels).
    width = 600
    height = 600

    # Setup visualization and user input
    window = pyglet.window.Window(width, height, visible=False)
    keyboard = key.KeyStateHandler()
    window.push_handlers(keyboard)
    draw_options = pymunk.pyglet_util.DrawOptions()
    draw_options.flags = draw_options.DRAW_SHAPES

    # Help text
    helpLabel = pyglet.text.Label(
        'ESC: quit, arrow-keys: move, d: debug on, c: autonomous control',
        font_size=12,
        x=window.width//2, y=window.height//20,
        anchor_x='center', anchor_y='center')

    # Show the number of collisions
    collisionLabel = pyglet.text.Label(
        str(collisions),
        font_size=18,
        x=window.width//2, y=window.height - 30,
        anchor_x='center', anchor_y='center')

    # build simulation environment
    env = pymunk.Space()
    env.damping = 0.01 # 99% of velocity is lost per second

    # Walls of the environment.
    wall_thickness = 10
    env_b = env.static_body
    walls = [
        pymunk.Segment(env_b, Vec2d(0, 0), Vec2d(width, 0), \
                       wall_thickness),
        pymunk.Segment(env_b, Vec2d(width, 0), Vec2d(width, height), \
                       wall_thickness),
        pymunk.Segment(env_b, Vec2d(width, height), Vec2d(0, height), \
                       wall_thickness),
        pymunk.Segment(env_b, Vec2d(0, height), Vec2d(0,0), \
                       wall_thickness)
    ]
    env.add(walls)

    # A few randomly distributed walls.
    random_walls = []
    n = randint(10, 20)
    for i in range(n):
        x1, y1 = randint(0, width), randint(0, height)
        angle = pi/2. * randint(0,3)
        length = randint(wall_thickness, width/2)
        x2, y2 = x1 + length*cos(angle), y1 + length*sin(angle)
        random_walls.append(
            pymunk.Segment(env.static_body, Vec2d(x1,y1), Vec2d(x2,y2), \
                           wall_thickness))
    env.add(random_walls)
    
    # Create the robots
    robots = []
    for i in range(N_ROBOTS):
        robot = DiffDriveBot()
        robot.body.position = width/2, height/2
        env.add(robot.body, robot.shape)

        # Create the range scanner
        robot.scanner = RangeScanner()

        # Create the controller
        #robot.controller = SimpleAvoider()
        robot.controller = RVOAvoider()

        robots.append(robot)

    def collision_handler(arbiter, space, data):
        global collisions
        collisions += 1
        collisionLabel.text = str(collisions)
        return True

    # Setup to handle collisions
    env.add_default_collision_handler().begin = collision_handler

    # define how to draw the visualization
    @window.event
    def on_draw():
        # always clear and redraw for graphics programming
        window.clear()
        env.debug_draw(draw_options)
        helpLabel.draw()
        collisionLabel.draw()
        if debug:
            for robot in robots:
                scan = robot.scanner.compute(env, robot, True)
                robot.controller.process_scan(robot, scan, True)

    def handle_keys():
        global allow_control, debug, steps_since_toggle

        manual_twist = Twist()

        if keyboard[key.RIGHT]:
            manual_twist.angular = -0.25 * MAX_ANGULAR_SPEED
        if keyboard[key.LEFT]:
            manual_twist.angular = 0.25 * MAX_ANGULAR_SPEED
        if keyboard[key.UP]:
            manual_twist.linear = 0.25 * MAX_LINEAR_SPEED
        if keyboard[key.DOWN]:
            manual_twist.linear = -0.25 * MAX_LINEAR_SPEED

        # Handle the keyboard events which shouldn't be toggled too quickly.
        steps_since_toggle += 1
        if keyboard[key.D] and steps_since_toggle > STEPS_BETWEEN_TOGGLES:
            debug = not debug
            steps_since_toggle = 0
        if keyboard[key.C] and steps_since_toggle > STEPS_BETWEEN_TOGGLES:
            allow_control = not allow_control
            steps_since_toggle = 0

        return manual_twist

    def update(dt):
        manual_twist = handle_keys()

        for robot in robots:
            update_for_robot(dt, robot, manual_twist)

    def update_for_robot(dt, robot, manual_twist):

        # First do autonomous control
        scan = robot.scanner.compute(env, robot, manual_twist)
        #print scan.ranges
        controller_twist = robot.controller.process_scan(robot, scan)
        twist = copy.deepcopy(manual_twist)
        if allow_control:
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

    # Update the simulation, allow for user input, and autonomous control
    for robot in robots:
        pyglet.clock.schedule_interval(robot.control_step, 1.0/120)
    pyglet.clock.schedule_interval(update, 1.0/60)
    pyglet.clock.schedule_interval(env.step, 1.0/60)

    # start simulation
    window.set_visible(True)
    pyglet.app.run()


# make module runnable from command line
if __name__ == '__main__':
    run()
