# Measurements

On an Apple M1 MacBook Air (macOS 27), 2026-10-14. `make test` and tools/bench.py print them again on
another machine; the arm's runs need an arm.

## Against LeRobot's RobotKinematics (tools/bench.py)

`uv run --with 'placo>=0.9.6,<0.9.16' --with 'cmeel-urdfdom>=4,<5' --with 'cmeel-tinyxml2<11'
tools/bench.py --lerobot CLONE --lerobot061 FILE`: placo 0.9.15 with LeRobot's pins, LeRobot's solver
from its main branch (kinematics.py as of ae2b57f12, 2026-10-13: 8 placo Newton steps a call) and from
LeRobot 0.6.1 (one step a call), so101ik's drop-in RobotKinematics; the same joints and targets, the
calls interleaved in one process, the median of 5 rounds:

| call | LeRobot main | LeRobot 0.6.1 | so101ik | faster |
|---|---|---|---|---|
| forward_kinematics, 500 random poses | 2.92 us | | 0.33 us | 8.8 x |
| inverse_kinematics, position, from the grid point 2 cm away (495) | 60.2 us | 11.2 us | 1.46 us | 41.3 x, 7.7 x |
| the same, max_iters=1 | 11.5 us | 11.2 us | 0.88 us | 13.2 x, 12.8 x |
| position, from the middle of every range (266) | 59.9 us | 11.3 us | 1.92 us | 31.3 x, 5.9 x |
| position and orientation, weights 1 and 0.01 (495) | 59.4 us | 11.1 us | 3.04 us | 19.5 x, 3.6 x |

An earlier run of the same code gave 3.7 to 42.5 times: the times move by a few percent from one
run to the next.

- Where the tip ended: from 2 cm away, so101ik within 1 mm at 100 % (LeRobot main 100 %, 3.0 mm at
  most; 0.6.1 at 56 %, 23.9 mm at most); after one step so101ik 0.47 mm off in the median. From the
  middle of every range, up to 38 cm away: so101ik 100 % (its fallback, the closed-form pose at the
  arm's pitch), LeRobot main 97 % and 327.8 mm off at most, 0.6.1 2 %. With the orientation held:
  so101ik within 1 mm at 71 % and 7.4 mm off at most, LeRobot main 69 % and 243.7 mm (an
  orientation the 5-joint arm cannot take there costs both the position; where both converge they
  end in the same joints).
- The forward kinematics agree to 1.1e-15.
- LeRobot's call sets the joints it is given but not placo's kinematics, so its first step works from
  the frames the call before left. Called from unrelated poses one after the other, LeRobot 0.6.1
  then misses by 270 mm in the median; bench.py puts placo's frames at the start pose before each of
  its calls (outside the time), as consecutive calls in teleoperation have them.
- so101ik's own search, the least-loaded allowed pose with the calibration's ranges, the table and
  the meshes: 17.0 microseconds a call over the grid, 3.5 times faster than LeRobot's call; the next
  pose of a path 5.5.

## The library alone, double and float (make test)

so101_ik over the 288 grid points at y 5 cm, wrist roll -90, zero offsets: 273 reached, mean score
0.204836 in double, float, and float with SO101_SMALL alike; the tip on its target to 2.6e-11 m in
double and 2e-6 m in float; every pose keeps the rules and the meshes. 2730 path steps of 2 mm:
neighbours at most 1.19 degrees apart in all three. Times in C (medians, a microsecond clock):
so101_ik 34-36 microseconds, so101_ik_near 10, so101_ik_local 2-3, so101_collides 0.34-0.36,
so101_pose_eval 0.12-0.14. Across runs they move by up to three times with the core a process gets
(AGENTS.md, Rules and traps), not with the precision.

## On the arm (media/triangle.gif, media/triangle.jpg)

examples/triangle.py on an SO-101 follower with 12 V STS3215 servos, with the leader arm on a second
USB port: unfold, move_to 20 12 18 cm, then move_by 8 cm left, 4 cm back and 7 cm up, 4 cm back and
7 cm down, at a peak of 5 cm/s, and home. SO101_PORT and SO101_CALIBRATION unset: Arm took the one
follower calibration and, of the two ports, the follower's (0.03 s); `check` found every servo's
EEPROM as in the calibration file. Offsets of 2.3, -5.9 and -3.0 degrees on lift, elbow and wrist
flex, measured for that arm.

- No stop, 29.9 s from torque on to torque off; ticks on fixed deadlines 20.0 ms apart (29 at most).
  The arm stood folded at torque-on and home() brought it back to that pose.
- The tip from the servos' positions against the target along each line (the model's tip both):
  5.0, 7.6 and 7.2 mm in the median, 3.1, 6.6 and 7.2 mm at the corners. The way up is the worst:
  the lift lags up to 4.7 degrees at 26 % load, the stick-slip of a loaded lift. An earlier run of
  the same moves: 5.7, 9.7 and 7.6 mm.
- One sync read of position, load and current of all six servos: 2.2 ms.
- media/triangle.gif: that run filmed by a camera beside the table, twice real speed;
  media/triangle.jpg: frames of the earlier run. In both the planned triangle and the tip's trail
  are drawn in through a calibrated camera model, from the servos' positions. The camera sees the
  plane x = 20 cm nearly edge-on.
