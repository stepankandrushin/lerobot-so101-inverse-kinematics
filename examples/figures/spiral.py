"""A spiral, drawn in the air with the gripper's tip as fast as the servos go.

    uv run examples/figures/spiral.py                # three laps, 24 cm wide, then the arm folds home
    uv run examples/figures/spiral.py --dry          # plan and print the timing, nothing moves
    uv run examples/figures/spiral.py --size 30 --laps 5 --learn 2

The coordinates below are the whole figure: in along a spiral of two turns and out along its mirror
image, back to the top; x to the right and y up as seen from in front, in any unit.
so101ik.figures scales it and does the rest (--help: the options).
"""

import numpy as np

from so101ik.figures import draw

T = np.linspace(0, 2 * np.pi, 1441)
IN = T < np.pi
S = np.where(IN, T / np.pi, 2 - T / np.pi)          # 0 at the rim, 1 at the middle, 0 again
POINTS = np.column_stack([(1 - S) * np.cos(4 * np.pi * S + np.pi / 2) * np.where(IN, 1, -1),
                          (1 - S) * np.sin(4 * np.pi * S + np.pi / 2)])

if __name__ == "__main__":
    draw("spiral", POINTS)
