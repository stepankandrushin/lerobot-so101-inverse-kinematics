# so101ik (lerobot-so101-inverse-kinematics)

The SO-101 follower arm (SO-ARM100/101, LeRobot's) in C: the least-loaded inverse kinematics, a local
solver that answers LeRobot's RobotKinematics calls, the mesh collision check, and the arm's driver
(Feetech STS servos, moves that stop on a stall or an overload). C99, no allocation, the board behind
a small HAL; a Python binding, `so101ik`, for macOS and Linux; an ESP32 next (agents/esp32.md).
README.md is for the people who use it, this file for whoever changes it.

**This repo is public.** Host names, addresses, the paths of one machine, and anything about one
particular arm, its room or its owner stay out of tracked files and commit messages. A git-ignored
CLAUDE.local.md may say where such notes live.

## Running

```sh
uv sync                                 # builds the C extension so101ik._core (again when a source changes)
make test                               # the C library alone, in double, float and float with SO101_SMALL
make example                            # examples/ik.c, the kinematics from C
uv run examples/kinematics.py           # the Python binding, no arm
uv run examples/triangle.py             # moves the arm: README.md, Safety
uv run tools/run_stats.py outputs/so101ik/NAME.csv    # how closely a run's tip followed its lines
uv run --with 'placo>=0.9.6,<0.9.16' --with 'cmeel-urdfdom>=4,<5' --with 'cmeel-tinyxml2<11' \
    tools/bench.py --lerobot CLONE [--lerobot061 FILE] [--urdf FILE]   # against LeRobot's solver
```

- The environment Arm reads: SO101_PORT, SO101_CALIBRATION and SO101_OFFSETS; without the first two
  it finds the port and LeRobot's calibration file itself (README.md, The port and the calibration).
- A run leaves outputs/so101ik/NAME.csv (every tick) and NAME.log under the directory it ran from;
  outputs/ is git-ignored.

## Layout

- include/so101ik.h: the whole C API. src/: kin.c (forward kinematics, torques, rules, collision
  check), ik.c (the IK and the local solver), sts.c and sts.h (the servos' packets), arm.c (the moves),
  hal_posix.c (the serial port on macOS and Linux), model.c (generated), so101ik_internal.h (what the
  sources share).
- python/so101ik/: __init__.py (Arm, Model, RobotKinematics, the port and calibration search),
  _core.c and _arm.c (the CPython binding, numpy's C API for LeRobot's arrays). setup.py builds them
  with src/.
- tools/: gen_model.py (src/model.c from the URDF and its meshes), bench.py, run_stats.py.
  test/test.c is make test. examples/: triangle.py (the arm), kinematics.py and ik.c (no arm).
- media/: the README's picture. .github/workflows/test.yml: make test, make example and the Python
  binding on Linux and macOS.

## Rules and traps

- src/model.c is generated: `make model` after the URDF or its meshes change, never an edit by hand.
  The URDF is SO-ARM100's Simulation/SO101/so101_new_calib.urdf: SO101_URDF, else ../SO-ARM100.
- After a change in src/: `make test` (single precision is what an ESP32 runs, and must give what
  double gives) and `make example`.
- Every exported C name starts with so101_ or SO101_: SO_ is the sockets' namespace (SO_REUSEADDR,
  SO_ERROR), which an ESP32 firmware with Wi-Fi includes.
- Arm never falls back to the calibration compiled into model.c (SO101_DEFAULT_CAL): it is another
  arm's, and the servos' EEPROM would refuse it. Model may, since it moves nothing.
- The offsets are 0 by default, the URDF as LeRobot takes it; the start pose is the arm's own
  (agents/design.md, The arm). Nothing in the library may assume one particular arm's numbers.
- find_port opens every candidate port when there are several and reads the servos' EEPROM: reads
  only, never a write, since one of them may be a leader arm or another device.
- uv rebuilds the extension when a .c or .h changes (cache-keys in pyproject.toml). The .so lands in
  python/so101ik/, git-ignored.
- The IK keeps its samples in static memory and its plane in the model: one thread at a time.
- On macOS a process can run on an efficiency core, and a timing then differs by up to 3 times from
  one run to the next: compare speeds in one process with the calls interleaved, as bench.py does.
- One program on a servo bus at a time: LeRobot's and this driver's packets collide.

## Docs

- agents/design.md: how the model, the IK, the collision check, the local solver and the driver
  work. Read it before changing src/.
- agents/measurements.md: against LeRobot's solver, float against double, a run on the arm.
- agents/esp32.md: the plan for an ESP32: which one, memory and time, the HAL, the bus, commands over
  Wi-Fi, Bluetooth or buttons.

## Editing rules

Current state goes in this file and agents/, in plain sentences with the reason for each rule; what
was done when, and why, in commit messages. Keep this file under 12 KB: a rule here in a sentence or
two, its long form in agents/.
