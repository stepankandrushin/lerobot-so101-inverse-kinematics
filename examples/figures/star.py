"""A five-pointed star, drawn in the air with the gripper's tip as fast as the servos go.

    uv run examples/figures/star.py                  # three laps, 24 cm wide, then the arm folds home
    uv run examples/figures/star.py --dry            # plan and print the timing, nothing moves
    uv run examples/figures/star.py --size 30 --laps 5 --learn 2

The coordinates below are the whole figure: its five points in the order the tip visits them, x to
the right and y up as seen from in front, in any unit. so101ik.figures scales it and does the rest
(--help: the options).
"""

import math

from so101ik.figures import draw

POINTS = [(math.cos(math.radians(90 + 144 * k)), math.sin(math.radians(90 + 144 * k))) for k in range(5)]

if __name__ == "__main__":
    draw("star", POINTS)
