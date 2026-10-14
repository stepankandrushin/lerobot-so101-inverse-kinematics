"""Fast runs: a path of the tip planned whole before the arm moves, then replayed as fast as the servos go.

    from so101ik import Arm
    from so101ik.fast import closed_run
    with Arm() as arm:
        run = closed_run(arm.model, points, laps=3)    # points: N x 3, m, a closed path 2 mm apart
        arm.run(run)            # unfolds at the run's pace, three laps; leaving the block folds it at that pace

The planning is the C library's (src/path.c): the joints at every point, each pose from the one before, the
gripper's pitch as asked ("fast": the smooth pitch along the lap that makes it shortest; "least": the
least-loaded poses; degrees, one number or one for each point); the fastest timing with every joint under
SETTINGS' vmax and amax; the run as joints every 10 ms, checked against the model. The driving
(src/fast.c): goals at 100 Hz that lead each joint by how it lags, the servos at full torque and without
their own ramp. agents/design.md, Fast runs, has the method; agents/measurements.md the numbers.
"""

import time
from dataclasses import dataclass, field

import numpy as np

from . import _core
from ._core import Fault, Unreachable

SETTINGS = _core.fast_defaults()   # vmax, amax, top, dt, lag, dead, smooth, gap, err_max, tube, torque_limit, acceleration
STEP = 0.002                       # m between the points of a path
FOLDED = 3.0                       # degrees from the start pose within which the arm counts as folded
GAIN = 0.7                         # of a replay's error that the next one corrects
JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")


@dataclass
class Run:
    """A run as the arm will be driven: where each joint and the tip should be at every tick."""

    q: np.ndarray                  # N x 6, calibrated degrees (the gripper 0-100), a tick every dt
    tips: np.ndarray               # N x 3, m
    parts: list = field(default_factory=list)   # what is drawn when: dicts with name, t0, t1 (s) and more
    dt: float = SETTINGS["dt"]

    @property
    def t(self) -> np.ndarray:
        return np.arange(len(self.q)) * self.dt

    @property
    def duration(self) -> float:
        return (len(self.q) - 1) * self.dt


def _settings(settings: dict | None) -> dict:
    return dict(SETTINGS, **(settings or {}))


def _tips(model, q: np.ndarray) -> np.ndarray:
    return np.array([model.tip(row)[:3] for row in q], float)


def joint_path(model, pts: np.ndarray, pitch="fast", roll: float = 0.0, gripper: float = 0.0,
               settings: dict | None = None) -> np.ndarray:
    """The joints (N x 6) that take the tip through pts (N x 3, m), each pose from the one before, every part
    the settings' gap from the others and the table. pitch: "fast", "least", or degrees (one number, or one
    for each point: of the two elbow branches that get round, the one whose lap is shorter). Raises
    Unreachable."""
    s = _settings(settings)
    pts = np.ascontiguousarray(pts, float)
    if isinstance(pitch, str):
        if pitch == "fast":
            return _core.path_fastest(model, pts, roll, gripper, s)[1]
        if pitch == "least":
            return _core.path_poses(model, pts, None, roll, gripper, 1, s["gap"])
        raise ValueError(f"pitch: 'fast', 'least' or degrees, not {pitch!r}")
    pitches = np.ascontiguousarray(np.broadcast_to(np.asarray(pitch, float), len(pts)))
    closed = np.allclose(pts[0], pts[-1])
    best, why = None, []
    for branch in (1, -1):
        try:
            q = _core.path_poses(model, pts, pitches, roll, gripper, branch, s["gap"])
        except Unreachable as e:
            why.append(f"elbow branch {branch:+d}: {e}")
            continue
        t = _core.path_time(q, 3 if closed else 1, s)
        if best is None or t[-1] < best[0]:
            best = (t[-1], q)
    if best is None:
        raise Unreachable("; ".join(why))
    return best[1]


def closed_run(model, pts, laps: int = 1, pitch="fast", speed: float = 1.0, roll: float = 0.0, gripper: float = 0.0,
               name: str = "path", settings: dict | None = None) -> Run:
    """A closed path of tip points (N x 3, m, a few mm apart, the last the first again), laps times round
    without a stop, as a run that starts and ends at rest at its first point. speed: 1 is the limits."""
    s = _settings(settings)
    pts = np.ascontiguousarray(pts, float)
    if len(pts) < 3 or np.linalg.norm(pts[0] - pts[-1]) > 1e-6:
        raise Fault(f"{name} does not end where it starts")
    q = joint_path(model, pts, pitch, roll, gripper, s)
    t = _core.path_time(q, laps, s)
    ticks, at = _core.path_sample(q, t, laps, speed, s)
    lap = len(pts) - 1
    k = np.minimum(np.floor(at).astype(int), lap * laps - 1)
    frac = (at - k)[:, None]
    tips = pts[k % lap] + frac * (pts[k % lap + 1] - pts[k % lap])
    times = np.arange(len(ticks)) * s["dt"]
    marks = [float(np.interp(i * lap, at, times)) for i in range(laps + 1)]
    v = np.linalg.norm(np.diff(tips, axis=0), axis=1) / s["dt"]
    part = {"name": name, "laps": laps, "pitch": pitch if isinstance(pitch, str) else "given", "t0": 0.0,
            "t1": float(times[-1]), "laps_at": marks, "lap_s": float(np.diff(marks).min()), "tip_peak": float(v.max()),
            "length": float(np.linalg.norm(np.diff(pts, axis=0), axis=1).sum()),
            "joint_peak": [float(x) for x in np.abs(np.diff(ticks[:, :5], axis=0)).max(axis=0) / s["dt"]]}
    return Run(ticks, tips, [part], s["dt"])


def joint_run(model, q: np.ndarray, speed: float = 1.0, name: str = "move", settings: dict | None = None) -> Run:
    """A joint path (N x 6, points a degree or less apart) as a run at the limits, starting and ending at rest."""
    s = _settings(settings)
    q = np.ascontiguousarray(q, float)
    ticks, _ = _core.path_sample(q, _core.path_time(q, 1, s), 1, speed, s)
    return Run(ticks, _tips(model, ticks), [{"name": name, "t0": 0.0, "t1": (len(ticks) - 1) * s["dt"]}], s["dt"])


def straight(a, b, step: float = 0.5) -> np.ndarray:
    """The straight joint path from a to b (six angles each), step degrees a point at most."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    n = max(2, int(np.abs(b - a).max() / step) + 1)
    return a + np.linspace(0, 1, n)[:, None] * (b - a)


def join(model, runs: list[Run], pause: float = 0.25, settings: dict | None = None) -> Run:
    """Runs one after the other as one: between two, a straight joint move from where one ends to where the
    next begins, timed like the rest, and pause seconds standing still before each. The whole ends where the
    first began, so it can be replayed."""
    s = _settings(settings)
    dt = s["dt"]
    q, tips, parts = [runs[0].q[:1]], [runs[0].tips[:1]], []
    for r in runs + [None]:
        to = runs[0].q[0] if r is None else r.q[0]
        if np.abs(to - q[-1][-1]).max() > 0.05:
            move = joint_run(model, straight(q[-1][-1], to), settings=s)
            q.append(move.q[1:])
            tips.append(move.tips[1:])
        if r is None:
            break
        hold = max(0, int(round(pause / dt)))
        q.append(np.repeat(r.q[:1], hold, axis=0))
        tips.append(np.repeat(r.tips[:1], hold, axis=0))
        now = sum(len(x) for x in q) * dt
        q.append(r.q)
        tips.append(r.tips)
        for p in r.parts:
            parts.append(dict(p, t0=p["t0"] + now, t1=p["t1"] + now,
                              laps_at=[x + now for x in p.get("laps_at", [])]))
    return Run(np.vstack(q), np.vstack(tips), parts, dt)


def check(model, run: Run, settings: dict | None = None) -> None:
    """The run against the model as it will be driven: each arm joint 2 degrees inside its range and under the
    servos' top speed, every part the gap from the others and the table. Raises Fault."""
    _core.run_check(model, np.ascontiguousarray(run.q, float), _settings(settings))


def describe(run: Run) -> str:
    """A line for each part of a run: its laps, length, time, fastest lap and peak speeds."""
    out = []
    for p in run.parts:
        size = f"{100 * p['size']:.0f} cm, " if "size" in p else ""
        if "laps" not in p:
            out.append(f"  {p['name']}: {p['t1'] - p['t0']:.2f} s")
            continue
        out.append(f"  {p['name']}: {size}{p['laps']} lap(s) of {100 * p['length']:.1f} cm in {p['t1'] - p['t0']:.2f} s, "
                   f"the fastest {p['lap_s']:.2f} s, the tip up to {100 * p['tip_peak']:.0f} cm/s, joints up to "
                   + " ".join(f"{x:.0f}" for x in p["joint_peak"]) + " degrees/s")
    return "\n".join(out)


def errors(model, run: Run, pos: np.ndarray, load: np.ndarray) -> dict:
    """How closely a replay followed its run, from the servos' readings: each arm joint's error (degrees), the
    tip's distance from where it should have been at that moment and from the path whatever the moment (mm),
    and each arm joint's peak load (%)."""
    tips = _tips(model, pos)
    timed = 1000 * np.linalg.norm(tips - run.tips, axis=1)
    path = run.tips[:: max(1, len(run.tips) // 3000)]
    off = np.array([np.linalg.norm(path - p, axis=1).min() for p in tips]) * 1000
    e = run.q[:, :5] - pos[:, :5]
    return {"tip_rms": float(np.sqrt((timed ** 2).mean())), "tip_max": float(timed.max()),
            "path_rms": float(np.sqrt((off ** 2).mean())), "path_max": float(off.max()),
            "joint_rms": np.sqrt((e ** 2).mean(axis=0)), "joint_max": np.abs(e).max(axis=0),
            "load_max": np.abs(load[:, :5]).max(axis=0)}


def report(e: dict) -> str:
    return (f"the tip {e['tip_rms']:.1f} mm rms and {e['tip_max']:.1f} at most from where it should be at each moment, "
            f"{e['path_rms']:.1f} and {e['path_max']:.1f} from the path; joints rms "
            + " ".join(f"{x:.1f}" for x in e["joint_rms"]) + ", at most " + " ".join(f"{x:.1f}" for x in e["joint_max"])
            + " degrees; peak loads " + " ".join(f"{x:.0f}" for x in e["load_max"]) + " %")


# ---- driving: what Arm.run and Arm's way home call ---------------------------------------------------------

def _pose(d: dict) -> np.ndarray:
    return np.array(list(d.values()), float)


def _folded(arm) -> bool:
    """The arm stands in its start pose, folded, and has not moved since the torque came on."""
    raw = arm.raw
    return bool(raw.folded and np.abs(_pose(raw.goal) - _pose(raw.start_pose))[:5].max() <= FOLDED)


def ways(arm, run: Run, settings: dict | None = None, speed: float = 1.0) -> tuple[Run | None, Run | None]:
    """The way from the start pose to the run's first point and from its last point back, as runs at speed of
    the limits (1 at most), where the model clears them with lift, elbow and wrist flex the settings' tube
    off; else None."""
    s = _settings(settings)
    start = _pose(arm.raw.start_pose)
    out = []
    for a, b, name in ((start, run.q[0], "unfold"), (start, run.q[-1], "fold")):
        if not _core.unfold_clear(arm.model, a, b, s["tube"]):
            out.append(None)
            continue
        q = _core.unfold_path(a, b)
        out.append(joint_run(arm.model, q if name == "unfold" else q[::-1], min(1.0, speed), name, s))
    return out[0], out[1]


def drive(arm, run: Run, learn: int = 0, name: str = "run", speed: float = 1.0, settings: dict | None = None) -> dict:
    """Arm.run: from where the arm stands to the run's first point (at speed of the limits from the folded
    start pose when the way is clear, else at the slow moves' pace), learn replays that correct the goals,
    and the run that counts. The arm stays at the run's end with the servos at the run's settings; Arm's way
    home folds it at the same pace (fold_home). Returns how the run went (errors)."""
    s = _settings(settings)
    raw, model = arm.raw, arm.model
    check(model, run, s)
    unfold, _ = ways(arm, run, s, speed) if _folded(arm) else (None, None)
    if unfold is not None:
        raw.fast(True, s)
        arm._fast = True
        raw.play(unfold.q, unfold.tips, rest=0.0, err_max=s["tube"], step="unfold", settings=s)
    else:
        arm._calm()
        arm._ready()
        raw.move_joints(dict(zip(JOINTS, run.q[0])), speed=60, step="to the first point")
        time.sleep(0.5)
        raw.fast(True, s)
        arm._fast = True
    arm._ran = None
    fix = None
    pos, load = np.zeros_like(run.q), np.zeros_like(run.q)
    for k in range(learn + 1):
        last = k == learn
        raw.play(run.q, run.tips, fix, pos, load, rest=0.1 if last else 0.3, step=name if last else f"learning {k + 1}",
                 settings=s)
        e = errors(model, run, pos, load)
        raw.log(("run: " if last else f"learning run {k + 1}: ") + report(e))
        if not last:
            fix = _core.path_learn(run.q, pos, fix, GAIN, s)
            time.sleep(0.7)
    arm._ran, arm._pace = run, speed
    return e


def fold_home(arm, settings: dict | None = None) -> bool:
    """After a run, back into the start pose at the run's pace and the torque off, if the arm still stands
    where the run ended and the way is clear; the servos' settings back as they were. Returns whether it did."""
    s = _settings(settings)
    run, arm._ran = arm._ran, None
    if run is None or not arm._fast or np.abs(_pose(arm.raw.goal) - run.q[-1])[:5].max() > 0.5:
        return False
    _, fold = ways(arm, run, s, getattr(arm, "_pace", 1.0))
    if fold is None:
        return False
    arm.raw.play(fold.q, fold.tips, rest=0.3, err_max=s["tube"], step="fold", settings=s)
    arm._calm()
    arm.raw.torque_off()
    return True
