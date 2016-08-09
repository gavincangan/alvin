import logging
import pyglet
import pymunk
import pymunk.pyglet_util

from math import pi

from ..planning import turn_and_go_path
from ..common import Path, Pose, Twist, Vector, M_TO_PIXELS
from ..common.math import normalize_angle
from ..common.util import isclose
from ..robots import DiffDriveBot

LINEAR_SPEED = 0.4 * M_TO_PIXELS
ANGULAR_VELOCITY = 0.2 * 2 * pi
LINEAR_TOLERANCE = 0.02 * M_TO_PIXELS
ANGULAR_TOLERANCE = (2 * pi)/200

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def run():
    # setup visualization and user input
    window = pyglet.window.Window(width=600, height=600, visible=False)

    # AV:
    draw_options = pymunk.pyglet_util.DrawOptions()
    draw_options.flags = draw_options.DRAW_SHAPES

    # help text
    label = pyglet.text.Label('ESC: quit, click to move',
                              font_size=20,
                              x=window.width//2, y=window.height//20,
                              anchor_x='center', anchor_y='center')

    robot = DiffDriveBot()
    robot.body.position = 300, 300
    env = pymunk.Space()
    env.add(robot.body, robot.shape)

    global path_start
    path_start = robot.get_pose().position
    global current_plan
    current_plan = Path()
    global current_pose_target
    current_pose_target = None

    @window.event
    def on_draw():
        window.clear()
        label.draw()
        if current_pose_target is not None:
            # draw line to show path
            pyglet.graphics.draw(2, pyglet.gl.GL_LINES,
                                 ('v2f', (path_start.position.x,
                                          path_start.position.y,
                                          current_pose_target.position.x,
                                          current_pose_target.position.y)),
                                 ('c3B', (0, 255, 0, 0, 255, 0)))
        # AV
        #pymunk.pyglet_util.draw(env)  # draw entire environment
        env.debug_draw(draw_options)

    @window.event
    def on_mouse_press(x, y, button, mod):
        global current_plan
        global current_pose_target

        start = robot.get_pose()
        goal = Pose(x=x, y=y, theta=None)
        current_plan = turn_and_go_path(start, goal,
                                        LINEAR_TOLERANCE, ANGULAR_TOLERANCE)
        current_pose_target = None
        logger.info("Current plan and target updated")

    def compute_command(dt):
        global current_pose_target
        global path_start
        if current_pose_target is None:
            robot.stop()
            if current_plan.length() > 0:
                current_pose_target = current_plan.next_pose()
                logger.info("New target set to: {}".format(
                    current_pose_target))
                path_start = robot.get_pose()
            return

        command = Twist()
        curr_pose = robot.get_pose()

        displacement = Vector(current_pose_target.position.x -
                              curr_pose.position.x,
                              current_pose_target.position.y -
                              curr_pose.position.y)
        logger.debug("Displacement: {}".format(displacement))
        at_x = isclose(displacement.x, 0., abs_tol=LINEAR_TOLERANCE)
        at_y = isclose(displacement.y, 0., abs_tol=LINEAR_TOLERANCE)

        curr_theta_normalized = normalize_angle(curr_pose.orientation)
        goal_theta_normalized = normalize_angle(
            current_pose_target.orientation)
        at_orientation = isclose(curr_theta_normalized,
                                 goal_theta_normalized,
                                 abs_tol=ANGULAR_TOLERANCE)

        if at_x and at_y and at_orientation:
            robot.stop()
            current_pose_target = None
        else:
            if at_x and at_y:
                command.linear.x = 0.
            else:
                # DiffDriveBot only moves linearly in x
                command.linear.x = LINEAR_SPEED

            if at_orientation:
                command.angular = 0.
            else:
                theta_delta = 0
                # determine which direction is the shortest turn
                if goal_theta_normalized > curr_theta_normalized:
                    theta_delta = goal_theta_normalized - curr_theta_normalized
                    command.angular = ANGULAR_VELOCITY if theta_delta < pi else -ANGULAR_VELOCITY
                else:
                    theta_delta = curr_theta_normalized - goal_theta_normalized
                    command.angular = ANGULAR_VELOCITY if theta_delta > pi else -ANGULAR_VELOCITY

            robot.set_command(command)

    pyglet.clock.schedule_interval(robot.control_step, 1.0/120)
    pyglet.clock.schedule_interval(compute_command, 1.0/60)
    pyglet.clock.schedule_interval(env.step, 1.0/60)

    # start simulation
    window.set_visible(True)
    pyglet.app.run()


# make module runnable from command line
if __name__ == '__main__':
    run()
