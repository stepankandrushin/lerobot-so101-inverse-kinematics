"""Figures in the air: what the scripts in examples/ call to draw with the gripper's tip as fast as the servos go.

    from so101ik.figures import draw
    draw("heart", POINTS)         # an example's last line: reads the options, plans, drives the arm

A figure is its coordinates and nothing else: a closed curve as x, y pairs in any unit, x to the right and y up
as seen from in front of it. Up to 20 points are the corners of a polygon (straight sides, every corner kept,
the last joined to the first); more are a curve, which must end where it starts. draw() scales the figure to
--size, lays it in a plane that faces --facing (a camera, or whoever watches) 30 cm from the shoulder, puts
a point every 2 mm along it, and has so101ik.fast plan the run and drive the arm round it.

show() is the same for several figures one after the other in one run, perform() lays the jaws and the
wrist's roll over a list of them (examples/dance.py).
"""

import argparse
import math
import sys
import time

import numpy as np

from . import Arm, Fault, Model, Unreachable
from .fast import Run, check, closed_run, describe, join, ways

SHOULDER = np.array([0.0388, 0.0, 0.1166])     # m: the pan axis at the height of the lift axis
FACING = (1.5, 0.0, 0.75)                       # m: whom the figures face by default, 1.5 m in front, 75 cm up
DISTANCE = 0.30                                 # m from SHOULDER to the plane's centre, towards FACING
RAISE = 0.01                                    # m: the figures' centre above that point, in the plane
PITCHES = {"fast": "the gripper's pitch that makes the lap shortest", "least": "the least-loaded poses",
           "facing": "the gripper pointing at the height of --facing"}


class Plane:
    """The plane the figures are drawn in: through centre, facing the point facing. right and up are its unit
    vectors as seen from there."""

    def __init__(self, facing=FACING, distance: float = DISTANCE, lift: float = RAISE):
        self.facing = np.asarray(facing, float)
        towards = self.facing - SHOULDER
        self.towards = towards / np.linalg.norm(towards)
        self.right = np.cross(-self.towards, [0, 0, 1.0])
        self.right /= np.linalg.norm(self.right)
        self.up = np.cross(self.right, -self.towards)
        self.centre = SHOULDER + distance * self.towards + lift * self.up

    def at(self, ab) -> np.ndarray:
        """Points in the base frame (m) of plane coordinates (N x 2, m)."""
        ab = np.atleast_2d(np.asarray(ab, float))
        return self.centre + ab[:, :1] * self.right + ab[:, 1:2] * self.up

    def pitch(self, p) -> float:
        """The pitch at which a gripper at p points at the height of facing: its elevation from p."""
        v = self.facing - np.asarray(p, float)
        return math.degrees(math.atan2(v[2], math.hypot(v[0], v[1])))


def outline(points, size: float, step: float = 0.002) -> np.ndarray:
    """A figure's coordinates as points in the plane (m): centred, its larger side size, a point every step
    along it, the first point again at the end. Up to 20 coordinates are a polygon, its sides divided and its
    corners kept."""
    p = np.asarray(points, float)
    polygon = len(p) <= 20
    if polygon and not np.allclose(p[0], p[-1]):
        p = np.vstack([p, p[:1]])
    lo, hi = p.min(axis=0), p.max(axis=0)
    p = (p - (lo + hi) / 2) / (hi - lo).max() * size
    if polygon:
        out = [p[0]]
        for a, b in zip(p, p[1:]):
            n = max(1, math.ceil(np.linalg.norm(b - a) / step))
            out += [a + (b - a) * k / n for k in range(1, n + 1)]
        return np.array(out)
    if not np.allclose(p[0], p[-1], atol=1e-9 * size):
        raise Fault("a figure of more than 20 points must end where it starts")
    s = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))])
    at = np.linspace(0, s[-1], max(2, math.ceil(s[-1] / step)) + 1)
    return np.column_stack([np.interp(at, s, p[:, k]) for k in range(2)])


def figure(model, plane: Plane, name: str, points, size: float, laps: int = 3, pitch="fast", speed: float = 1.0,
           roll: float = 0.0, gripper: float = 0.0) -> Run:
    """One figure, size m wide or high (its larger side), laps times round without a stop, as a run that starts
    and ends at rest at its first point."""
    pts = plane.at(outline(points, size))
    if pitch == "facing":
        pitch = np.array([plane.pitch(p) for p in pts])
    run = closed_run(model, pts, laps, pitch, speed, roll, gripper, name)
    run.parts[0].update(size=size, pitch=pitch if isinstance(pitch, str) else "facing")
    return run


def dance(model, plane: Plane, steps: list[tuple], speed: float = 1.0, roll: float = 0.0, gripper: float = 0.0) -> Run:
    """Figures as one run, the jaws snapping open at the start of each and the wrist rolling through those that
    say so (which moves the tip a few millimetres off its figure: the tip is not on the roll axis). steps:
    (name, coordinates, size in cm, laps, speed, whether the wrist rolls)."""
    runs = [figure(model, plane, name, points, size / 100, laps, "fast", speed * s, roll, gripper)
            for name, points, size, laps, s, _ in steps]
    run = join(model, runs, pause=0.15)
    t = run.t
    vmax = 220.0
    for p, (*_, rolls) in zip(run.parts, steps):
        m = (t >= p["t0"] - 1e-9) & (t <= p["t1"] + 1e-9)
        x = t[m] - p["t0"]
        # The jaws open in 0.45 s and close in another 0.45 (the gripper's servo does about 115 % a second).
        run.q[m, 5] = gripper + 50 * np.clip(np.minimum(x, 0.9 - x) / 0.45, 0, 1)
        if rolls:
            dur = p["t1"] - p["t0"]
            period = dur / max(1, round(dur / 1.3))
            swing = min(45.0, 0.7 * vmax * period / (2 * np.pi))
            run.q[m, 4] = roll + swing * np.clip(np.minimum(x, dur - x) / 0.3, 0, 1) * np.sin(2 * np.pi * x / period)
    run.tips = np.array([model.tip(row)[:3] for row in run.q], float)
    return run


# ---- the examples' command lines --------------------------------------------------------------------------

def _parser(name: str, shape: bool = True) -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=sys.modules["__main__"].__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    if shape:
        ap.add_argument("--size", type=float, default=24, help="cm, the figure's larger side (24)")
        ap.add_argument("--laps", type=int, default=3, help="(3)")
        ap.add_argument("--pitch", default="fast",
                        help="; ".join(f"{k}: {v}" for k, v in PITCHES.items()) + "; or degrees (fast)")
    ap.add_argument("--speed", type=float, default=1.0, help="1 is as fast as the limits allow (1)")
    ap.add_argument("--learn", type=int, default=0, help="replays before the run, each correcting the goals by what the "
                    "joints were off in the one before (0)")
    ap.add_argument("--facing", type=float, nargs=3, default=FACING, metavar=("X", "Y", "Z"),
                    help="m in the arm's base frame: the point the figures face, a camera say (1.5 0 0.75: in front)")
    ap.add_argument("--roll", type=float, help="the wrist roll, degrees (as the arm stands; 0 with --dry)")
    ap.add_argument("--gripper", type=float, help="the gripper, 0-100 (as the arm stands; 0 with --dry)")
    ap.add_argument("--dry", action="store_true", help="plan and print; nothing moves, the arm is not opened")
    ap.add_argument("--name", default=time.strftime(f"{name}-%Y%m%d-%H%M%S"), help="the run's files in outputs/so101ik/")
    return ap


def _pitch(value: str):
    if value in PITCHES:
        return value
    try:
        return float(value)
    except ValueError:
        sys.exit(f"--pitch: {' '.join(PITCHES)} or degrees, not {value}")


def _go(a: argparse.Namespace, build) -> None:
    """Plan (build(model, plane, roll, gripper) gives the run), print it, and unless --dry drive the arm."""
    plane = Plane(a.facing)

    def plan(model, roll, gripper) -> Run:
        try:
            run = build(model, plane, roll, gripper)
            check(model, run)
        except (Unreachable, Fault) as e:
            sys.exit(f"Not possible as asked: {e}. Nothing moved.")
        print(f"{run.duration:.1f} s, the plane's centre at {np.round(100 * plane.centre, 1)} cm, facing "
              f"{np.round(plane.facing, 2)} m; the wrist roll {roll:+.1f}, the gripper {gripper:.0f}")
        print(describe(run))
        return run

    if a.dry:
        plan(Model(), a.roll or 0.0, a.gripper or 0.0)
        return
    with Arm(name=a.name) as arm:
        here = arm.raw.goal
        run = plan(arm.model, here["wrist_roll"] if a.roll is None else a.roll,
                   here["gripper"] if a.gripper is None else a.gripper)
        unfold, fold = ways(arm, run, speed=a.speed) if arm.raw.folded else (None, None)
        print(f"  from the start pose {unfold.duration:.2f} s, back into it {fold.duration:.2f} s" if unfold and fold
              else "  from where it stands and back at the slow moves' pace")
        arm.run(run, learn=a.learn, name=run.parts[0]["name"] if len(run.parts) == 1 else "figures", speed=a.speed)


def draw(name: str, points) -> None:
    """An example's command line: the figure called name, with these coordinates."""
    a = _parser(name).parse_args()
    pitch = _pitch(a.pitch)
    _go(a, lambda model, plane, roll, grip: figure(model, plane, name, points, a.size / 100, a.laps, pitch, a.speed,
                                                   roll, grip))


def show(figures: dict) -> None:
    """examples/show.py's command line: several of figures (name: coordinates) one after the other in one run.
    Each is a name, or name:speed:laps:pitch:size with any of them left empty."""
    ap = _parser("show")
    ap.add_argument("figures", nargs="+", metavar="FIGURE",
                    help=f"of {' '.join(figures)}; or name:speed:laps:pitch:size with any of them left empty "
                         "(circle:0.3:1 circle::3:least), to mix them in one run")
    a = ap.parse_args()
    specs = [spec.split(":") + [""] * 4 for spec in a.figures]
    if unknown := [f[0] for f in specs if f[0] not in figures]:
        sys.exit(f"no figure called {' or '.join(unknown)}: there are {' '.join(figures)}")
    _go(a, lambda model, plane, roll, grip: join(model, [
        figure(model, plane, f[0], figures[f[0]], float(f[4] or a.size) / 100, int(f[2] or a.laps), _pitch(f[3] or a.pitch),
               float(f[1] or 1) * a.speed, roll, grip) for f in specs]))


def perform(steps: list[tuple]) -> None:
    """examples/dance.py's command line: dance(steps)."""
    a = _parser("dance", shape=False).parse_args()
    _go(a, lambda model, plane, roll, grip: dance(model, plane, steps, a.speed, roll, grip))
