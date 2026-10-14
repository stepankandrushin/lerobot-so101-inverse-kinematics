"""What a run's CSV says about each step: the ticks, the tip's distance from the target it was sent to
(both by the model, the tip from the servos' positions), the joints' lag behind their goals and their
loads. Reads the CSV that so101ik's Arm writes (outputs/so101ik/NAME.csv).

    uv run tools/run_stats.py outputs/so101ik/triangle.csv [more.csv ...]
"""
import csv
import sys

import numpy as np

ARM = ("shoulder_lift", "elbow_flex", "wrist_flex")

for path in sys.argv[1:]:
    rows = list(csv.DictReader(open(path)))
    t = np.array([float(r["unix_time"]) for r in rows])
    print(f"{path}: {len(rows)} ticks over {t[-1] - t[0]:.1f} s, {1000 * np.median(np.diff(t)):.1f} ms apart (median), "
          f"{1000 * np.diff(t).max():.1f} at most")
    for step in dict.fromkeys(r["step"] for r in rows):
        rs = [r for r in rows if r["step"] == step]
        if not step.startswith("line"):
            continue
        tip = np.array([[float(r[f"tip_{a}"]) for a in "xyz"] for r in rs])
        tgt = np.array([[float(r[f"target_{a}"]) for a in "xyz"] for r in rs])
        d = np.linalg.norm(tip - tgt, axis=1)
        lag = [max(abs(float(r[f"{j}_goal"]) - float(r[f"{j}_pos"])) for r in rs) for j in ARM]
        load = [max(abs(float(r[f"{j}_load"])) for r in rs) for j in ARM]
        print(f"   {step:24s} {len(rs):4d} ticks: the tip {np.median(d):5.1f} mm from its target (median), {d.max():5.1f} at "
              f"most, {d[-1]:4.1f} at the end; lag at most " + " ".join(f"{v:.1f}" for v in lag)
              + " degrees, load " + " ".join(f"{v:.0f}" for v in load) + " % (lift, elbow, wrist flex)")
