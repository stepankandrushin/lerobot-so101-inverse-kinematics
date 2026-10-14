"""Several figures one after the other in one run: one unfolding, one folding.

    uv run examples/figures/show.py circle square heart --laps 3
    uv run examples/figures/show.py circle heart star --learn 2 --name three
    uv run examples/figures/show.py circle:0.3:1 circle::3:least circle::4:fast --dry    # one figure, three ways

A figure is named as its script here, or as name:speed:laps:pitch:size with any of them left empty,
which then come from the options. A new script joins by its line in FIGURES.
"""

import circle
import clover
import flower
import heart
import infinity
import spiral
import square
import star
import triangle
import waves

from so101ik.figures import show

FIGURES = {m.__name__: m.POINTS
           for m in (circle, square, triangle, star, infinity, heart, waves, spiral, clover, flower)}

if __name__ == "__main__":
    show(FIGURES)
