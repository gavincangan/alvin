import pyglet

def draw_square(pos, side_length, rgb, thickness):
    radius = side_length / 2
    x1 = pos[0] - radius
    y1 = pos[1] - radius
    x2 = pos[0] + radius
    y2 = pos[1] + radius
    pyglet.gl.glLineWidth(thickness)
    vertices = (x1, y1, x2, y1, x2, y2, x1, y2)
    colors = (rgb[0], rgb[1], rgb[2], rgb[0], rgb[1], rgb[2], \
              rgb[0], rgb[1], rgb[2], rgb[0], rgb[1], rgb[2])
    pyglet.graphics.draw(4, pyglet.gl.GL_QUADS,
        ('v2f', vertices),
        ('c3B', colors))
    pyglet.gl.glLineWidth(1)
