"""A triangle, drawn in the air with the gripper's tip as fast as the servos go.

    uv run examples/figures/triangle.py              # three laps, 24 cm wide, then the arm folds home
    uv run examples/figures/triangle.py --dry        # plan and print the timing, nothing moves
    uv run examples/figures/triangle.py --size 30 --laps 5 --learn 2

The coordinates below are the whole figure: its corners, x to the right and y up as seen from
in front, in any unit. so101ik.figures scales it and does the rest (--help: the options).
"""

import math

from so101ik.figures import draw

POINTS = [(math.cos(math.radians(a)), math.sin(math.radians(a))) for a in (90, 210, 330)]

if __name__ == "__main__":
    draw("triangle", POINTS)
