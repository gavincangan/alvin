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

    # AV:
    space = pymunk.Space()
    draw_options = pymunk.pyglet_util.DrawOptions()
    draw_options.flags = draw_options.DRAW_SHAPES
    step_dt = 1/60.0

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

    # AV
    space.add(rob_body, rob_shape)

    # define how to draw the visualization
    @window.event
    def on_draw():
        # always clear and redraw for graphics programming
        window.clear()
        label.draw()

        # AV
        space.debug_draw(draw_options)

    # use keyboard to control Rob
    def update(dt):
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

        space.step(step_dt)

    # process at regular intervals
    pyglet.clock.schedule_interval(update, step_dt)

    # start simulation
    window.set_visible(True)
    pyglet.app.run()


# make module runnable from command line
if __name__ == '__main__':
    run()
