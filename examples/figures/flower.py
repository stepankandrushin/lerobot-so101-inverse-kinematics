"""A flower of five petals, drawn in the air with the gripper's tip as fast as the servos go.

    uv run examples/figures/flower.py                # three laps, 24 cm wide, then the arm folds home
    uv run examples/figures/flower.py --dry          # plan and print the timing, nothing moves
    uv run examples/figures/flower.py --size 30 --laps 5 --learn 2

The coordinates below are the whole figure: a closed curve, x to the right and y up as seen from
in front, in any unit. so101ik.figures scales it and does the rest (--help: the options).
"""

import numpy as np

from so101ik.figures import draw

T = np.linspace(0, 2 * np.pi, 1441)
R = 0.6 + 0.4 * np.cos(5 * T)
POINTS = np.column_stack([R * np.sin(-T), R * np.cos(T)])

if __name__ == "__main__":
    draw("flower", POINTS)
