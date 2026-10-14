"""so101ik against LeRobot's own kinematics (lerobot.model.kinematics.RobotKinematics, built on placo):
the same calls from the same joints to the same targets, timed in one process, one after the other.
No hardware.

    uv run --with 'placo>=0.9.6,<0.9.16' --with 'cmeel-urdfdom>=4,<5' --with 'cmeel-tinyxml2<11' \\
        tools/bench.py [--lerobot CLONE] [--lerobot061 FILE] [--urdf FILE] [--repeat N]

LeRobot's file is loaded as it stands in a clone of its repo (--lerobot, else ../lerobot beside this
repo: 8 placo Newton steps a call) and, if given, as LeRobot 0.6.1 installs it (--lerobot061, its
lerobot/model/kinematics.py: one step a call); placo with LeRobot's own pins. The URDF is --urdf, else
SO101_URDF, else ../SO-ARM100/Simulation/SO101/so101_new_calib.urdf. Both solvers get URDF degrees and
the same 4x4 targets; so101ik's RobotKinematics is the drop-in one (so101ik/__init__.py). LeRobot's call sets the joints
it is given but not placo's kinematics, so its first Newton step works from the frames the call before
left; in teleoperation those are the start's, and here forward_kinematics(start) puts them there
before each of its calls, outside the time. Times are the median of
--repeat rounds of each call (the call alone, by perf_counter_ns), and the tip's distance from its
target is measured afterwards with so101ik's forward kinematics, which agrees with placo's.
"""

import argparse
import gc
import importlib.util
import math
import os
import sys
import time
import types
from pathlib import Path

import numpy as np

import so101ik

ROOT = Path(__file__).resolve().parents[1]
URDF = Path(os.environ.get("SO101_URDF", ROOT.parent / "SO-ARM100/Simulation/SO101/so101_new_calib.urdf")).expanduser()
XS, ZS = np.arange(0.10, 0.41, 0.02), np.arange(0.02, 0.37, 0.02)


def stock(path: Path):
    """LeRobot's RobotKinematics from its file. Of the package around it the file needs one function, which
    checks that placo is installed."""
    for name in ("lerobot", "lerobot.utils", "lerobot.utils.import_utils"):
        sys.modules.setdefault(name, types.ModuleType(name))
    sys.modules["lerobot.utils.import_utils"].require_package = lambda *a, **k: None
    spec = importlib.util.spec_from_file_location(f"lerobot_kinematics_{abs(hash(str(path)))}", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.RobotKinematics(str(URDF), "gripper_frame_link", so101ik.JOINTS[:5])


def timed(f, *args, **kw):
    t = time.perf_counter_ns()
    r = f(*args, **kw)
    return r, time.perf_counter_ns() - t


def us(ns) -> str:
    v = float(np.median(ns)) / 1000
    return f"{v:.2f} us" if v < 1000 else f"{v / 1000:.2f} ms"


def tip_error(q, target) -> float:
    return 1000 * float(np.linalg.norm(so101ik.fk_urdf(np.asarray(q, float))[:3, 3] - target[:3, 3]))


def main() -> None:
    global URDF
    ap = argparse.ArgumentParser()
    ap.add_argument("--lerobot", type=Path, default=ROOT.parent / "lerobot",
                    help="a clone of LeRobot (its src/lerobot/model/kinematics.py)")
    ap.add_argument("--lerobot061", type=Path, help="LeRobot 0.6.1's kinematics.py, one Newton step a call")
    ap.add_argument("--urdf", type=Path, default=URDF, help="SO-ARM100's so101_new_calib.urdf")
    ap.add_argument("--repeat", type=int, default=5)
    a = ap.parse_args()
    URDF = a.urdf.expanduser()
    if not URDF.is_file():
        raise SystemExit(f"{URDF}: no URDF; clone https://github.com/TheRobotStudio/SO-ARM100 beside this repo, "
                         "or give --urdf")
    clone_file = a.lerobot.expanduser() / "src/lerobot/model/kinematics.py"
    if not clone_file.is_file():
        raise SystemExit(f"{clone_file}: no LeRobot; clone https://github.com/huggingface/lerobot beside this repo, "
                         "or give --lerobot")
    solvers = {"LeRobot (repo, 8 steps)": stock(clone_file)}
    if a.lerobot061:
        solvers["LeRobot 0.6.1 (1 step)"] = stock(a.lerobot061.expanduser())
    ours = so101ik.RobotKinematics()
    model = so101ik.Model()
    rng = np.random.default_rng(0)
    gc.disable()
    rows = []

    def say(s=""):
        print(s, flush=True)

    say(f"so101ik against LeRobot's RobotKinematics on this machine; median of {a.repeat} rounds per call")

    # forward kinematics
    lo = np.array([so101ik.Model(stock=True).ranges[j][0] for j in range(5)])
    hi = np.array([so101ik.Model(stock=True).ranges[j][1] for j in range(5)])
    qs = [rng.uniform(lo, hi) for _ in range(500)]
    lr = next(iter(solvers.values()))
    apart = max(float(np.abs(np.asarray(lr.forward_kinematics(q)) - ours.forward_kinematics(q)).max()) for q in qs)
    t_lr, t_us = [], []
    for _ in range(a.repeat):
        for q in qs:
            t_lr.append(timed(lr.forward_kinematics, q)[1])
            t_us.append(timed(ours.forward_kinematics, q)[1])
    say(f"forward_kinematics, 500 random poses: the frames {apart:.1e} apart at most; LeRobot {us(t_lr)}, "
        f"so101ik {us(t_us)}: {np.median(t_lr) / np.median(t_us):.1f} times")
    rows.append(("forward_kinematics", np.median(t_lr), np.median(t_us)))

    # the grid as LeRobot's teleoperation reaches it: from the middle of every range, 5 mm at a time
    mid = np.zeros(5)
    eye = np.eye(4)
    sol = {}
    q = mid.copy()
    for i, x in enumerate(XS):
        for j, z in enumerate(ZS):
            T = eye.copy()
            T[:3, 3] = x, 0.0, z
            r = so101ik.Model(stock=True)
            try:
                p = r.ik(x, 0.0, z, roll=0.0)
            except so101ik.Unreachable:
                continue
            sol[(i, j)] = np.array(p.joints[:5])
    pairs = [(k, n) for k in sol for n in ((k[0] + 1, k[1]), (k[0], k[1] + 1)) if n in sol]

    def target(k, rot=None):
        T = np.eye(4)
        if rot is not None:
            T[:3, :3] = rot
        T[:3, 3] = XS[k[0]], 0.0, ZS[k[1]]
        return T

    def scenario(title, starts, weight, iters=None, held=False):
        say(f"\n{title}, {len(starts)} calls")
        times = {name: [] for name in list(solvers) + ["so101ik"]}
        errs = {name: [] for name in times}
        for rnd in range(a.repeat):
            for q0, T in starts:
                for name, s in list(solvers.items()) + [("so101ik", ours)]:
                    kw = {} if iters is None or name.startswith("LeRobot 0.6.1") else {"max_iters": iters}
                    if s is not ours:
                        # LeRobot's call sets the joints but not placo's kinematics, so its first step works from
                        # the frames the call before left: as in teleoperation, they are the start's.
                        s.forward_kinematics(q0)
                    q, t = timed(s.inverse_kinematics, q0.copy(), T, 1.0, weight, **kw)
                    times[name].append(t)
                    if rnd == 0:
                        errs[name].append(tip_error(q, T))
        base = np.median(times["so101ik"])
        for name in times:
            e = np.array(errs[name])
            say(f"   {name:24s} {us(times[name]):>10s} a call ({np.median(times[name]) / base:5.1f} x so101ik's); the tip "
                f"{np.median(e):.3f} mm from the target (median), within 1 mm at {np.mean(e < 1):.0%}, {e.max():.1f} mm at most")
        for name in solvers:
            rows.append((f"{title} vs {name}", np.median(times[name]), base))

    near = [(sol[k], target(n)) for k, n in pairs]
    scenario("inverse_kinematics, position alone, from the pose of the grid point beside it (2 cm)", near, 0.0)
    scenario("the same, one Newton step a call (max_iters=1)", near, 0.0, iters=1)
    far = [(mid.copy(), target(k)) for k in sol]
    scenario("position alone, from the middle of every range (up to 38 cm away)", far, 0.0)
    # held: LeRobot's default weights, the gripper's orientation of the start pose kept
    held = []
    for k, n in pairs:
        T0 = ours.forward_kinematics(sol[k])
        held.append((sol[k], target(n, T0[:3, :3])))
    scenario("position and orientation (LeRobot's default weights 1 and 0.01), from the grid point beside it", held, 0.01)

    # so101ik's own: the least-loaded pose of all that reach the point, with every check
    say("\nso101ik's own inverse kinematics on the same grid (the calibration's ranges, the table and the meshes)")
    t_best, t_near = [], []
    pts = [(XS[k[0]], 0.05, ZS[k[1]]) for k in sol]
    for _ in range(a.repeat):
        for p in pts:
            try:
                _, t = timed(model.ik, *p, roll=-90.0)
                t_best.append(t)
            except so101ik.Unreachable:
                pass
    prev = None
    for p in pts:
        try:
            prev = model.ik(*p, roll=-90.0)
        except so101ik.Unreachable:
            continue
        for k in range(1, 6):
            try:
                _, t = timed(model.ik, prev.tip[0], prev.tip[1] + 0.002 * k, prev.tip[2], near=prev)
                t_near.append(t)
            except so101ik.Unreachable:
                break
    lr_call = float(np.median([t for name, t, _ in rows if name.startswith("inverse_kinematics, position alone") and "repo" in name] or [math.nan]))
    say(f"   Model.ik(x, y, z): the least-loaded allowed pose, {len(pts)} points: {us(t_best)} a call; LeRobot's call above "
        f"took {lr_call / np.median(t_best):.1f} times as long")
    say(f"   Model.ik(..., near=pose): the next pose of a path, 2 mm on: {us(t_near)}")

    say("\nspeed-up, LeRobot's time over so101ik's:")
    for name, t_lr, t_us in rows:
        say(f"   {t_lr / t_us:6.1f} x  {name}")


if __name__ == "__main__":
    main()
