import pyglet
import pymunk
import pymunk.pyglet_util

from pyglet.window import key
from ..common import M_TO_PIXELS


def run():
    # setup visualization and user input
    window = pyglet.window.Window(width=600, height=600, visible=False)
    keyboard = key.KeyStateHandler()
    window.push_handlers(keyboard)

    # help text
    label = pyglet.text.Label('ESC: quit, arrow-keys: move',
                              font_size=20,
                              x=window.width//2, y=window.height//20,
                              anchor_x='center', anchor_y='center')

    # build Rob the robot
    rob_mass = 1  # 1 kg
    rob_radius = 0.1 * M_TO_PIXELS  # 0.1 meter radius, converted to pixels for display
    rob_I = pymunk.moment_for_circle(rob_mass, 0, rob_radius)  # moment of inertia for disk
    rob_body = pymunk.Body(rob_mass, rob_I)
    rob_body.position = 300, 300
    rob_shape = pymunk.Circle(rob_body, rob_radius)
    rob_shape.color = 255, 0, 0  # red

    # define how to draw the visualization
    @window.event
    def on_draw():
        # always clear and redraw for graphics programming
        window.clear()
        label.draw()
        pymunk.pyglet_util.draw(rob_shape)  # this is gold right here

    # use keyboard to control Rob
    def process_user_input(dt):
        speed = 0.6  # m/s
        distance = speed * dt * M_TO_PIXELS  # m/s * s -> pixels

        # direction
        if keyboard[key.RIGHT]:
            rob_body.position += distance, 0
        if keyboard[key.LEFT]:
            rob_body.position -= distance, 0
        if keyboard[key.UP]:
            rob_body.position += 0, distance
        if keyboard[key.DOWN]:
            rob_body.position -= 0, distance

    # process input at regular interval
    pyglet.clock.schedule_interval(process_user_input, 1.0/60)

    # start simulation
    window.set_visible(True)
    pyglet.app.run()


# make module runnable from command line
if __name__ == '__main__':
    run()
