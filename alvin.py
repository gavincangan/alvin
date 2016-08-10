import pyglet
import pymunk
import pymunk.pyglet_util

from math import pi, cos, sin, sqrt
from pyglet.window import key
from pymunk import Vec2d

from common import Twist, M_TO_PIXELS
from robots import DiffDriveBot

LINEAR_SPEED = 0.1 * M_TO_PIXELS
ANGULAR_SPEED = 0.2 * 2 * pi

def unit_direction_vector(theta=0.0):
    return cos(theta), sin(theta)

def run():
    # setup visualization and user input
    window = pyglet.window.Window(width=600, height=600, visible=False)
    keyboard = key.KeyStateHandler()
    window.push_handlers(keyboard)

    # AV:
    draw_options = pymunk.pyglet_util.DrawOptions()
    draw_options.flags = draw_options.DRAW_SHAPES

    # help text
    label = pyglet.text.Label('ESC: quit, arrow-keys: move',
                              font_size=10,
                              x=window.width//2, y=window.height//20,
                              anchor_x='center', anchor_y='center')

    # build simulation environment
    env = pymunk.Space()
    env.damping = 0.5

    # obstacles
    static_lines = [
        pymunk.Segment(env.static_body, Vec2d(10,10), Vec2d(590,10), 5),
        pymunk.Segment(env.static_body, Vec2d(590,10), Vec2d(590,590), 5),
        pymunk.Segment(env.static_body, Vec2d(590,590), Vec2d(10,590), 5),
        pymunk.Segment(env.static_body, Vec2d(10,590), Vec2d(10,10), 5)
    ]
    env.add(static_lines)
    


    # build Rob the robot
    robot = DiffDriveBot()
    robot.body.position = 300, 300
    env.add(robot.body, robot.shape)

    # define how to draw the visualization
    @window.event
    def on_draw():
        # always clear and redraw for graphics programming
        window.clear()
        label.draw()

        range_scan(True)

        # AV
        env.debug_draw(draw_options)

    # use keyboard to control Rob
    def process_user_input(dt):
        twist = Twist()

        # direction
        if keyboard[key.RIGHT]:
            twist.angular = -ANGULAR_SPEED
        if keyboard[key.LEFT]:
            twist.angular = ANGULAR_SPEED
        if keyboard[key.UP]:
            twist.linear.x = LINEAR_SPEED
            #robot.body.apply_impulse_at_local_point((force, 0), (0,0))
        if keyboard[key.DOWN]:
            twist.linear.x = -LINEAR_SPEED
            #robot.body.apply_impulse_at_local_point((-force, 0), (0,0))

        robot.set_command(twist)

    def range_scan(visualize=False):
        sensor_start_radius = robot.radius + 5
        sensor_max_range = 100
        hits = []
        for sensor_angle in [-pi/3., -pi/6., 0, pi/6., pi/3.]:
            c = cos(robot.body.angle + sensor_angle)
            s = sin(robot.body.angle + sensor_angle)
            x1 = int(robot.body.position.x + sensor_start_radius * c)
            y1 = int(robot.body.position.y + sensor_start_radius * s)
            x2 = int(robot.body.position.x + sensor_max_range * c)
            y2 = int(robot.body.position.y + sensor_max_range * s)

            query_info = env.segment_query_first((x1, y1), (x2, y2), 1, [])
            if query_info == None or query_info.shape == None:
                hits.append(sensor_max_range)

                if visualize:
                    pyglet.graphics.draw(2, pyglet.gl.GL_LINES,
                                 ('v2f', (x1, y1, x2, y2)),
                                 ('c3B', (255, 0, 0, 255, 0, 0)))
            else:
                hits.append(query_info.alpha * sensor_max_range)

                if visualize:
                    pyglet.graphics.draw(2, pyglet.gl.GL_LINES,
                                 ('v2f', (x1, y1, x2, y2)),
                                 ('c3B', (0, 255, 0, 0, 255, 0)))
        return hits

    def autonomous_control(dt):
        hits = range_scan()
        print hits

    # Update the simulation, allow for user input, and autonomous control
    pyglet.clock.schedule_interval(robot.control_step, 1.0/120)
    pyglet.clock.schedule_interval(process_user_input, 1.0/60)
    pyglet.clock.schedule_interval(autonomous_control, 1.0/60)
    pyglet.clock.schedule_interval(env.step, 1.0/60)

    # start simulation
    window.set_visible(True)
    pyglet.app.run()


# make module runnable from command line
if __name__ == '__main__':
    run()
