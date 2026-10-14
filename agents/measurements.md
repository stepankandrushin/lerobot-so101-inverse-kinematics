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

## On the arm: the triangle (media/triangle.jpg)

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
- media/triangle.jpg: frames of the earlier run, filmed by a camera beside the table, the planned
  triangle and the tip's trail drawn in through a calibrated camera model from the servos'
  positions. The camera sees the plane x = 20 cm nearly edge-on.

## Fast runs

### The limits (the same follower, 12 V STS3215, 12.4 V at the servos)

Measured before this library, with the Python driver it was ported from, on 2026-10-08:

- A goal step on one joint at a time from the arm stretched out, read at 380 Hz: peak 166-197
  degrees/s at Torque_Limit 500 (127 for the lift and elbow going up), 254-304 at 1000. Torque_Limit
  caps the drive (the load reads 50 % throughout a step at 500): it halves the speed as well as the
  force. The data sheet says 270 degrees/s at 12 V.
- Each of pan, lift, elbow and wrist flex following a sinusoid of +-30 degrees streamed at 100 Hz
  (periods 2.0 to 0.8 s, up to 236 degrees/s and 1850 degrees/s^2) at Torque_Limit 1000: with the
  servo's own ramp (Acceleration 254) the joint lags 97-121 ms, more the faster; with Acceleration 0
  74-86 ms at every frequency, which a first-order lag of 60 ms behind a dead time of 22 ms fits
  (amplitude 0.89-0.92 at 1.25 Hz). Peak loads 74-93 % at 0.8 s.
- A circle followed up to 253 degrees/s (1.15 times the limits); at 286 the shoulder stood at full
  drive and the tip was 7.6 mm rms off. Hence vmax 220, amax 1500, top 295.

### Planning (`uv run examples/figures/NAME.py --dry`, an M1)

The ten figures at 24 cm, three laps, the pitch that makes a lap shortest, facing a viewer 1.5 m in
front (within 0.03 s the same facing a camera 1.6 m away, 56 degrees to the arm's left), the wrist
roll at -89:

| figure | lap | tip up to | | figure | lap | tip up to |
|---|---|---|---|---|---|---|
| circle | 0.74 s | 127 cm/s | | heart | 1.68 s | 88 cm/s |
| infinity | 1.23 s | 91 cm/s | | clover | 2.26 s | 91 cm/s |
| triangle | 1.49 s | 99 cm/s | | star | 2.53 s | 93 cm/s |
| square | 1.85 s | 95 cm/s | | spiral | 2.82 s | 105 cm/s |
| flower | 3.16 s | 77 cm/s | | waves | 3.38 s | 57 cm/s |

- Each plans in 20-60 ms (the whole `--dry` run, Python's start included, in 0.1-0.2 s). The Python
  driver it was ported from took about 20 s for the same search and found the same lap: a 20 cm heart
  in 1.47 s in both.
- A heart plans up to 32 cm across (1.89 s a lap, the tip at up to 100 cm/s), facing either way.
- make test: a 90-degree move of one joint takes 0.5558 s, the trapezoid d/v + v/a; a 20 cm circle's
  fastest lap is 0.684 s in double and 0.685 in float, its ticks 6 micrometres off the circle at most;
  on simulated servos the second replay follows 3 times closer than the first.

### On the arm: the heart (media/heart.gif, media/heart.jpg)

examples/figures/heart.py --size 30, facing the camera that filmed it (1.6 m away, 56 degrees to the
arm's left, 28 up), the gripper pitch that makes a lap shortest, on 2026-10-15. Offsets 2.3, -5.9
and -3.0 degrees on lift, elbow and wrist flex. The arm stood folded at torque-on and took the fast
way out and back each time.

| run | speed | lap | the tip from where it should be, rms / most | joints rms, at most | peak loads pan lift elbow wflex |
|---|---|---|---|---|---|
| half speed | 0.5 | 3.67 s | 3.6 / 11.1 mm | 0.4-0.6, 2.0 degrees | 32 42 37 36 % |
| filmed | 1 | 1.84 s | 4.1 / 8.7 mm | 0.7-1.0, 2.5 degrees | 63 74 67 73 % |
| learning, replay 1 | 1 | 1.84 s | 4.1 / 9.8 mm | 0.7-1.0, 2.4 degrees | 62 73 66 77 % |
| replay 2 | 1 | | 1.7 / 6.6 mm | 0.3-0.4, 1.2 degrees | 61 74 68 76 % |
| the run | 1 | | 1.2 / 5.4 mm | 0.1-0.2, 0.7 degrees | 61 74 68 76 % |

- The filmed run: 8.7 s from the command to the torque off; the torque on 0.5 s after the command,
  1.02 s out of the rest pose, three laps in 5.55 s (the tip at up to 93 cm/s), 1.02 s back. Ticks
  10.0 ms apart in the median, 14 at most, in all three parts.
- From the path whatever the moment, the filmed run's tip was 2.8 mm rms and 6.9 at most off; after
  two learning replays 0.9 and 2.3.
- The tip in these numbers is the model's at the servos' readings; gearbox play and bending parts
  are not in them. In the film the trail drawn from them lies on the blurred gripper.
