"""The dance: nine figures in a row without folding in between, the jaws snapping open at the start of
each and the wrist rolling through the smooth ones.

    uv run examples/figures/dance.py --dry
    uv run examples/figures/dance.py --learn 2
"""

from show import FIGURES

from so101ik.figures import perform

# The figure, its size in cm, its laps, its speed (1 is the limits), whether the wrist rolls along.
DANCE = [
    ("circle", 22, 2, 1.0, True),
    ("infinity", 32, 2, 1.0, True),
    ("star", 24, 1, 1.0, False),
    ("waves", 32, 1, 1.0, True),
    ("clover", 26, 1, 1.0, True),
    ("square", 18, 2, 1.0, False),
    ("triangle", 22, 2, 1.0, False),
    ("heart", 22, 1, 0.8, False),
    ("spiral", 24, 1, 1.0, True),
]

if __name__ == "__main__":
    perform([(name, FIGURES[name], *rest) for name, *rest in DANCE])
