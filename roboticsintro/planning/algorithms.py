import logging

from math import atan2, sqrt
from ..common import Pose, Path
from ..common.util import allclose, isclose

logger = logging.getLogger(__name__)


# TODO take closeness tolerance as argument (to match control resolution)
def turn_and_go_path(start, goal):
    """Find a straight-line path from start to goal.

    Find a path from start to goal consisting of a single rotation,
    a straight line, and a final rotation to match goal orientation.

    :param start: starting pose
    :type  start: roboticsintro.common.Pose
    :param goal: goal pose to find path to
    :type  goal: roboticsintro.common.Pose
    :returns: path from start to goal or empty path if goal is at start
    :rtype: roboticsintro.common.Path
    """
    path = Path()

    logger.debug("turn_and_go_path called with start: {}, goal: {}".format(start, goal))
    # check we're not already at the goal
    if not allclose(start.position, goal.position):
        dir_x, dir_y = (goal.position.x - start.position.x,
                        goal.position.y - start.position.y)
        logger.debug("direction vector: [{}, {}]".format(dir_x, dir_y))
        heading_theta = atan2(dir_y, dir_x)
        logger.debug("heading: {}".format(heading_theta))

        # check if we're already on the correct heading
        if not isclose(start.orientation, heading_theta):
            # first rotation part of plan
            path.append_pose(Pose(x=start.position.x,
                                  y=start.position.y,
                                  theta=heading_theta))

        # linear travel part of plan
        path.append_pose(Pose(x=goal.position.x,
                              y=goal.position.y,
                              theta=heading_theta))

    # check if we're already at goal orientation
    if not isclose(start.orientation, goal.orientation):
        # final rotation part of plan
        path.append_pose(Pose(x=goal.position.x,
                              y=goal.position.y,
                              theta=goal.orientation))

    logger.debug("returning {}".format(path))
    return path
