"""A heart, drawn in the air with the gripper's tip as fast as the servos go.

    uv run examples/figures/heart.py                 # three laps, 24 cm wide, then the arm folds home
    uv run examples/figures/heart.py --dry           # plan and print the timing, nothing moves
    uv run examples/figures/heart.py --size 30 --laps 5 --learn 2

The coordinates below are the whole figure: a closed curve, x to the right and y up as seen from
in front, in any unit. so101ik.figures scales it and does the rest (--help: the options).
"""

import numpy as np

from so101ik.figures import draw

T = np.linspace(0, 2 * np.pi, 1441)
POINTS = np.column_stack([-16 * np.sin(T) ** 3,
                          13 * np.cos(T) - 5 * np.cos(2 * T) - 2 * np.cos(3 * T) - np.cos(4 * T)])

if __name__ == "__main__":
    draw("heart", POINTS)
