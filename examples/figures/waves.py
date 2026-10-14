"""Two waves, drawn in the air with the gripper's tip as fast as the servos go.

    uv run examples/figures/waves.py                 # three laps, 24 cm wide, then the arm folds home
    uv run examples/figures/waves.py --dry           # plan and print the timing, nothing moves
    uv run examples/figures/waves.py --size 30 --laps 5 --learn 2

The coordinates below are the whole figure: there along a sine of two periods and back along its
mirror image, a chain of four eyes; x to the right and y up as seen from in front, in any unit.
so101ik.figures scales it and does the rest (--help: the options).
"""

import numpy as np

from so101ik.figures import draw

T = np.linspace(0, 2 * np.pi, 1441)
X = np.where(T < np.pi, -1 + 2 * T / np.pi, 3 - 2 * T / np.pi)
POINTS = np.column_stack([X, 0.5 * np.sin(2 * np.pi * (X + 1)) * np.where(T < np.pi, 1, -1)])

if __name__ == "__main__":
    draw("waves", POINTS)
