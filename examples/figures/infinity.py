"""A lying eight, drawn in the air with the gripper's tip as fast as the servos go.

    uv run examples/figures/infinity.py              # three laps, 24 cm wide, then the arm folds home
    uv run examples/figures/infinity.py --dry        # plan and print the timing, nothing moves
    uv run examples/figures/infinity.py --size 30 --laps 5 --learn 2

The coordinates below are the whole figure: a closed curve, x to the right and y up as seen from
in front, in any unit. so101ik.figures scales it and does the rest (--help: the options).
"""

import numpy as np

from so101ik.figures import draw

T = np.linspace(0, 2 * np.pi, 1441)
POINTS = np.column_stack([np.sin(T), np.sin(T) * np.cos(T)])

if __name__ == "__main__":
    draw("infinity", POINTS)
