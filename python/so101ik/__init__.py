"""so101ik: the SO-101 arm (SO-ARM100/101, LeRobot's follower) in C, from Python.

    from so101ik import Arm
    with Arm() as arm:                     # torque on, unfold from the start pose; folds home at the end
        arm.move_to(0.20, 0.15, 0.20)      # the tip to x, y, z in metres, in the least-loaded pose
        arm.move_by(0, 0.05, 0)            # a straight line from where the tip is

Model is the kinematics (forward, gravity torques, the least-loaded inverse kinematics, the rules and
the mesh collision check); RobotKinematics has LeRobot's two calls (lerobot.model.kinematics), as fast
as the C library makes them. Coordinates are the URDF's base frame in metres (x forward, y to the arm's
left, z up), angles LeRobot's calibrated degrees, the gripper 0 to 100.

Arm finds the arm by itself: LeRobot's calibration file of the follower (find_calibration) and the
serial port whose servos hold that calibration (find_port). The environment can say instead:

    SO101_PORT          the follower's serial port, e.g. /dev/tty.usbmodem58760431541 or /dev/ttyACM0; "sim" for
                        simulated servos, to try a program without an arm
    SO101_CALIBRATION   the follower's calibration file from lerobot-calibrate, a .json
    SO101_OFFSETS       degrees from calibrated to URDF angles, e.g. "shoulder_lift=2.3,elbow_flex=-5.9"
                        (0 for every joint when unset)
"""

import glob
import json
import os
import signal
import sys
import time
from pathlib import Path

from ._core import Arm as _Arm
from ._core import Fault, Pose, Unreachable, default_calibration, fk_urdf, ik_local
from ._core import Model as _Model
from . import fast

__all__ = ["Arm", "Model", "Pose", "RobotKinematics", "Unreachable", "Fault", "SetupError", "JOINTS",
           "find_port", "find_calibration", "read_calibration", "read_offsets"]

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
# Where lerobot-calibrate keeps a follower's file under the calibration root: so_follower/ from LeRobot
# 0.4 on (SO-100 and SO-101 alike), so101_follower/ before.
ROBOT_DIRS = ("so_follower", "so101_follower")
PORTS = {"darwin": ("/dev/tty.usbmodem*", "/dev/tty.usbserial*"), "linux": ("/dev/ttyACM*", "/dev/ttyUSB*")}


class SetupError(RuntimeError):
    """The port or the calibration file cannot be told: none, or several. The message says what to set."""


def calibration_root() -> Path:
    """LeRobot's calibration folder, found as LeRobot finds it: HF_LEROBOT_CALIBRATION, else
    HF_LEROBOT_HOME/calibration, else HF_HOME/lerobot/calibration (~/.cache/huggingface/lerobot/calibration)."""
    if v := os.environ.get("HF_LEROBOT_CALIBRATION"):
        return Path(v).expanduser()
    if v := os.environ.get("HF_LEROBOT_HOME"):
        return Path(v).expanduser() / "calibration"
    hf = os.environ.get("HF_HOME") or Path(os.environ.get("XDG_CACHE_HOME", "~/.cache")) / "huggingface"
    return Path(hf).expanduser() / "lerobot" / "calibration"


def find_calibration() -> Path:
    """SO101_CALIBRATION, else the one follower calibration that lerobot-calibrate wrote
    (calibration_root()/robots/so_follower/ID.json). None or several: SetupError."""
    if v := os.environ.get("SO101_CALIBRATION"):
        path = Path(v).expanduser()
        if not path.is_file():
            raise SetupError(f"SO101_CALIBRATION={v}: no such file")
        return path
    robots = calibration_root() / "robots"
    found = sorted(p for d in ROBOT_DIRS for p in (robots / d).glob("*.json"))
    if len(found) == 1:
        return found[0]
    if not found:
        raise SetupError(f"no follower calibration in {robots / ROBOT_DIRS[0]}: calibrate the arm with LeRobot "
                         "(lerobot-calibrate --robot.type=so101_follower --robot.port=PORT --robot.id=NAME), "
                         "or set SO101_CALIBRATION to its .json file")
    raise SetupError("several follower calibrations: " + ", ".join(map(str, found))
                     + "; set SO101_CALIBRATION to this arm's")


def read_calibration(path: Path | str) -> list[tuple[int, int, int, int]]:
    """(id, homing_offset, range_min, range_max) per joint, pan to gripper, from LeRobot's calibration file."""
    path = Path(path)
    try:
        cal = json.loads(path.read_text())
        return [(cal[j]["id"], cal[j]["homing_offset"], cal[j]["range_min"], cal[j]["range_max"]) for j in JOINTS]
    except (OSError, ValueError, KeyError, TypeError) as e:
        raise SetupError(f"{path}: not a calibration of an SO-101 follower ({type(e).__name__}: {e})") from None


def read_offsets(offsets=None) -> tuple[float, ...]:
    """The five offsets, pan to roll, in degrees (URDF angle = calibrated angle + offset): from a dict by joint
    name or a sequence of five; None reads SO101_OFFSETS ("shoulder_lift=2.3,elbow_flex=-5.9"), 0 where unset."""
    if offsets is None:
        offsets = {}
        for item in filter(None, (s.strip() for s in os.environ.get("SO101_OFFSETS", "").split(","))):
            name, sep, value = item.partition("=")
            try:
                offsets[name.strip()] = float(value)
            except ValueError:
                raise SetupError(f"SO101_OFFSETS: {item!r} is not joint=degrees") from None
    if isinstance(offsets, dict):
        unknown = set(offsets) - set(JOINTS[:5])
        if unknown:
            raise ValueError(f"offsets: unknown joints {sorted(unknown)}; the joints are {JOINTS[:5]}")
        return tuple(float(offsets.get(j, 0.0)) for j in JOINTS[:5])
    offsets = tuple(float(v) for v in offsets)
    if len(offsets) != 5:
        raise ValueError("offsets: five numbers, pan to roll, or a dict by joint name")
    return offsets


def _holds(port: str, cal) -> bool:
    """Whether the servos on port answer and hold cal in their EEPROM (read only, nothing moves)."""
    try:
        arm = _Arm(port, cal, log=lambda line: None)
    except Fault:
        return False
    try:
        arm.check()
        return True
    except Fault:
        return False
    finally:
        arm.close()


def find_port(calibration=None) -> str:
    """SO101_PORT, else the one USB serial port there is (macOS /dev/tty.usbmodem* or tty.usbserial*, Linux
    /dev/ttyACM* or ttyUSB*). With several (a leader arm too, say) and a calibration given, the one whose
    servos hold it in their EEPROM; the check only reads. None, or no single one: SetupError."""
    if v := os.environ.get("SO101_PORT"):
        return v
    patterns = PORTS.get(sys.platform, PORTS["linux"])
    found = sorted({p for pattern in patterns for p in glob.glob(pattern)})
    if len(found) == 1:
        return found[0]
    if not found:
        raise SetupError(f"no USB serial port ({', '.join(patterns)}): is the arm's driver board plugged in? "
                         "Or set SO101_PORT")
    if calibration is not None:
        holding = [p for p in found if _holds(p, calibration)]
        if len(holding) == 1:
            return holding[0]
        found_s = ", ".join(found)
        if not holding:
            raise SetupError(f"no servos with this calibration on {found_s}: is the follower powered? "
                             "Set SO101_PORT to its port, or SO101_CALIBRATION to its calibration")
        found = holding
    raise SetupError(f"several serial ports: {', '.join(found)}; set SO101_PORT to the follower's "
                     "(lerobot-find-port tells which)")


def ranges(cal) -> list[tuple[float, float]]:
    """Each arm joint's calibrated range in degrees, pan to roll."""
    out = []
    for _, _, lo, hi in cal[:5]:
        mid = (lo + hi) / 2
        out.append(((lo - mid) * 360 / 4095, (hi - mid) * 360 / 4095))
    return out


class Model(_Model):
    """The arm's kinematics: Model().ik(0.25, 0, 0.08, roll=-90) -> Pose. The ranges come from
    calibration_file, else from find_calibration() if it finds one, else from the calibration compiled in;
    offsets as read_offsets takes them. stock=True: LeRobot's view, no offsets, the URDF's limits, no table
    and no meshes."""

    def __init__(self, stock: bool = False, calibration_file: Path | str | None = None, offsets=None):
        super().__init__(stock=stock)
        if stock:
            return
        if calibration_file is None:
            try:
                calibration_file = find_calibration()
            except SetupError:
                if "SO101_CALIBRATION" in os.environ:
                    raise
        cal = read_calibration(calibration_file) if calibration_file else default_calibration()
        self.set_ranges(ranges(cal))
        self.offsets = read_offsets(offsets)


class RobotKinematics:
    """LeRobot's RobotKinematics (lerobot.model.kinematics) for the SO-101, compiled: the same two calls,
    URDF degrees in and out. The model is the library's (so101_new_calib.urdf), so urdf_path and
    target_frame_name are taken for the signature's sake; joint_names must be the five arm joints.

    inverse_kinematics stops when its steps are under 1e-9 rad (at most max_iters of them), and where it
    did not get the position within 1 mm it takes, with fallback, the pose at the arm's pitch from the
    library's closed-form solver. After each call, error is the tip's distance from the target (m) and
    steps the Gauss-Newton steps taken."""

    def __init__(self, urdf_path: str | None = None, target_frame_name: str = "gripper_frame_link",
                 joint_names: list[str] | None = None, fallback: bool = True):
        if target_frame_name != "gripper_frame_link":
            raise ValueError("the tip frame is gripper_frame_link")
        if joint_names is not None and list(joint_names)[:5] != JOINTS[:5]:
            raise ValueError(f"joint_names: {JOINTS[:5]}")
        self.joint_names = JOINTS[:5]
        self._fallback = Model(stock=True) if fallback else None
        self.error, self.steps = 0.0, 0

    def forward_kinematics(self, joint_pos_deg):
        """The tip frame (4x4) for URDF degrees, pan to roll (more entries are ignored)."""
        return fk_urdf(joint_pos_deg)

    def inverse_kinematics(self, current_joint_pos, desired_ee_pose, position_weight: float = 1.0,
                           orientation_weight: float = 0.01, max_iters: int = 8):
        """Joints (URDF degrees, the gripper and anything past the arm kept) that put the tip frame at
        desired_ee_pose, starting from current_joint_pos."""
        q, self.steps, self.error = ik_local(current_joint_pos, desired_ee_pose, position_weight, orientation_weight,
                                             max_iters, self._fallback)
        return q


class Arm:
    """The follower on its servo bus, for a with block: torque on at the start, and the first move unfolds the
    arm from its start pose (run() at the run's own pace, the others with the wrist first); folded home and
    torque off at the end, also after an error (a run at its own pace). Ctrl-C (or a kill, a hang-up) stops
    the move and the arm goes home; a second one switches the torque off, and the arm drops.

    port and calibration_file default to find_port() and find_calibration(), offsets to SO101_OFFSETS (see
    read_offsets). Every tick goes to OUT/NAME.csv, every event to OUT/NAME.log and stdout; on_tick(dict) is
    called at every tick of a move too (time, step, goal, pos, load, ma, tip, target), to watch it. Before
    a run: the arm folded in its start pose, nothing in its way.

    Without the with block, a program that folds the arm its own way calls check(), torque_on() and its moves
    itself, and torque_off() and close() at the end. Every move is one call with its data (a point, a chain
    of poses, the gripper's rate) that the C library runs whole, so the same calls can go to a board over
    a link. A move stops on a stall, an overload, Ctrl-C (KeyboardInterrupt) or stop() (Fault); the goals
    then stay where the joints stand."""

    def __init__(self, port: str | None = None, calibration_file: Path | str | None = None, offsets=None,
                 name: str | None = None, out: Path | str = Path("outputs") / "so101ik", on_tick=None):
        port = port or os.environ.get("SO101_PORT")
        try:
            calibration_file = Path(calibration_file) if calibration_file else find_calibration()
        except SetupError:
            if port != "sim":
                raise
            calibration_file = None        # the simulated servos take the calibration compiled in
        cal = read_calibration(calibration_file) if calibration_file else default_calibration()
        offsets = read_offsets(offsets)
        port = port or find_port(cal)
        out = Path(out)
        out.mkdir(parents=True, exist_ok=True)
        self.name = name or time.strftime("arm-%Y%m%d-%H%M%S")
        self._log = (out / f"{self.name}.log").open("a")
        self._echo = True
        self._arm = _Arm(port, cal, log=self.line, csv=str(out / f"{self.name}.csv"), on_tick=on_tick)
        self._arm.offsets = offsets
        self.model = Model(calibration_file=calibration_file, offsets=offsets) if calibration_file else Model(offsets=offsets)
        self.model.set_ranges(ranges(cal))      # the arm's own Model, for planning: the same ranges and offsets
        self._fast, self._ran, self._pace = False, None, 1.0     # a fast run's settings on; the run it ended; its pace
        self._signals = {}
        self._arm.log(f"port {port}, calibration {calibration_file}, offsets "
                      + " ".join(f"{v:+g}" for v in offsets))

    def line(self, text: str) -> None:
        """Each event line, with its time: into NAME.log and onto stdout. When stdout stops taking them (its
        terminal gone), they go to the log only and lost() is called, once."""
        self._log.write(text + "\n")
        self._log.flush()
        if self._echo:
            try:
                print(text, flush=True)
            except OSError:
                self._echo = False
                sys.stdout = open(os.devnull, "w")
                self.lost()

    def lost(self) -> None:
        """Called once when stdout stops taking the log. The arm goes on; a subclass may stop() it."""

    def _stop(self, *_):
        raise KeyboardInterrupt

    def __enter__(self) -> "Arm":
        try:
            self._arm.check()
            for sig in (signal.SIGTERM, signal.SIGHUP):
                self._signals[sig] = signal.signal(sig, self._stop)
            self._arm.torque_on()
        except BaseException:
            self.__exit__(None, None, None)
            raise
        return self

    def __exit__(self, kind, value, tb) -> bool:
        try:
            if self._arm.torque:
                if kind is not None:
                    self._arm.log(f"{kind.__name__}: {value}; going home")
                folded = False
                if kind is None and self._fast:
                    try:
                        folded = fast.fold_home(self)
                    except Fault as e:
                        self._arm.log(f"folding at the run's pace: {e}; going home at the slow moves' pace")
                if not folded:
                    self._calm()
                    self._home()
        except KeyboardInterrupt:
            self._arm.log("stopped again: torque off, the arm drops")
        except Fault as e:
            self._arm.log(f"on the way home: {e}; torque off")
        finally:
            try:
                if self._arm.torque:
                    self._arm.torque_off()
            finally:
                self._arm.close()
                self._log.close()
                for sig, old in self._signals.items():
                    signal.signal(sig, old)
        return False

    def _at_start(self) -> bool:
        """The torque is on and the arm still stands in the start pose it stood folded in at torque-on."""
        return bool(self._arm.torque and self._arm.folded and max(
            abs(self._arm.goal[j] - self._arm.start_pose[j]) for j in JOINTS[:5]) <= fast.FOLDED)

    def _calm(self) -> None:
        """The servos' settings back from a fast run's to the slow moves'."""
        if self._fast:
            self._arm.fast(False)
            self._fast = False

    unfold_first = True     # the first move unfolds the arm if it still stands folded; False for a program that
                            # unfolds it its own way (round things on the table, say)

    def _ready(self) -> None:
        """Before a move: the slow moves' settings, and the arm unfolded if it still stands folded (unfold_first)."""
        self._calm()
        self._ran = None
        if self.unfold_first and self._at_start():
            self._arm.unfold()

    def _home(self) -> None:
        if self._at_start():
            self._arm.torque_off()      # never moved: nothing to fold
        else:
            self._arm.home()

    def run(self, run, learn: int = 0, name: str = "run", speed: float = 1.0) -> dict:
        """A run planned whole (so101ik.fast.closed_run, so101ik.figures), as fast as the servos go: from the
        folded start pose to its first point at speed of the limits (from anywhere else at the slow moves'
        pace), learn replays that each correct the goals by what the joints were off in the one before, and
        the run that counts. The servos run at full torque meanwhile. Leaving the with block folds the arm
        from the run's end at the same pace. Returns how closely the run followed (so101ik.fast.errors)."""
        return fast.drive(self, run, learn, name, speed)

    def move_to(self, x: float, y: float, z: float, pitch: float | None = None, speed: float = 30.0) -> Pose:
        """The tip to (x, y, z) m, in the least-loaded allowed pose (or at pitch, degrees), straight in joint
        space at a peak of speed degrees/s."""
        self._ready()
        return self._arm.move_to(x, y, z, pitch, speed)

    def move_by(self, dx: float, dy: float, dz: float, speed: float = 0.05) -> Pose:
        """The tip dx, dy, dz m from where it is, along a straight line at a peak of speed m/s."""
        self._ready()
        return self._arm.move_by(dx, dy, dz, speed)

    def line_to(self, x: float, y: float, z: float, speed: float = 0.05) -> Pose:
        """The tip along a straight line to (x, y, z) m at a peak of speed m/s."""
        self._ready()
        return self._arm.line_to(x, y, z, speed)

    def move_joints(self, target, speed: float = 30.0, check: bool = True, step: str = "joints") -> None:
        """The joints in target (a dict by name, or all six) to their angles, straight in joint space. check:
        refused if the straight way runs past a range, near the table or into the arm itself."""
        self._ready()
        self._arm.move_joints(target, speed, check, step)

    def follow(self, poses, speed: float = 0.03, *, duration: float | None = None, steady: bool = False,
               settle: float = 0.3, within: float = 0.0, lead: bool = False, press: float = 0.0,
               free: tuple[float, float] = (0.15, 0.5), press_ticks: int = 3, step: str | None = None) -> bool:
        """Through a chain of poses (Poses, dicts by joint or six angles each; the gripper stays where it is
        where one does not say), linearly between them, at a peak tip speed in m/s or over duration s, on
        one minimum-jerk profile (steady: at a steady pace), then held settle s. The first pose must be
        within 5 degrees of where the arm was sent last, neighbours within 3 of each other, every pose inside
        the ranges; the chain is the caller's plan, nothing else is checked.

        within: the hold ends as soon as pan, lift, elbow and wrist flex stand this close (degrees) to the last
        pose. lead: each of them is led ahead of its pose by what it lags (the gearbox's friction leaves a
        servo up to 1.5 degrees short); the lead (Arm.lead) stays for the next chain. press (m): lowering until
        contact: the move stops once the tip, from the servos, stands this much higher above the goals' tip
        than it did in the free stretch (fractions of the duration) for press_ticks ticks; it ends there or at
        the chain's end with every goal where its joint stands, and returns whether it touched."""
        self._ready()
        return self._arm.follow(poses, speed, duration or 0.0, steady, settle, within, lead, press, free, press_ticks,
                                step)

    def grasp(self, rate: float = 40.0, past: float = 3.0, squeeze: float = 2.5, empty: float = 4.0,
              step: str = "grasp") -> float:
        """Close the gripper at rate percent/s until the jaw has stood still for 0.25 s with its goal past
        percent beyond it. Returns where it stopped, percent: under empty it holds nothing and stays there;
        else the goal goes squeeze past it, so it holds what it closed on."""
        self._calm()
        return self._arm.grasp(rate, past, squeeze, empty, step)

    def rise(self, z: float, speed: float = 0.01) -> None:
        """The tip straight up to height z (m) at speed m/s, without the table checks: the way out of a pose at
        the table. Fault if a joint reaches its range end first, the tip as high as it got."""
        self._calm()
        self._arm.rise(z, speed)

    def unfold(self) -> None:
        """From the start pose to it with the wrist straight (the first move does this itself)."""
        self._calm()
        self._arm.unfold()

    def home(self) -> None:
        """Fold to the start pose and switch the torque off."""
        self._calm()
        self._home()

    def read(self):
        """(degrees, load %, mA), each a dict by joint."""
        return self._arm.read()

    def check(self) -> None:
        """Every servo answers and holds the calibration in its EEPROM; Fault if not (nothing moves)."""
        self._arm.check()

    def torque_on(self) -> None:
        """Goals to where the joints stand, the RAM settings, torque on; the start pose is recorded."""
        self._arm.torque_on()

    def torque_off(self) -> None:
        """Torque off: the arm drops. Allowed from inside a move too (a second Ctrl-C's handler)."""
        self._arm.torque_off()

    def sim(self, floor: float | None = None, block: dict | None = None, pose=None) -> None:
        """The simulated arm's world (port "sim", or SO101_PORT=sim): a floor the tip stops at, m; joints
        blocked at calibrated angles, {"gripper": (12.0, None)} for an object that holds the jaws at 12 percent
        ({} for none); the servos put at a pose."""
        self._arm.sim(floor, block, pose)

    def stop(self) -> None:
        """The move running, or the next one, stops at its next tick and raises Fault. Any thread, a signal
        handler or the tick callable may ask."""
        self._arm.stop()

    def log(self, text: str) -> None:
        """A line into the run's log, with the time, as the arm's own go."""
        self._arm.log(text)

    def close(self) -> None:
        """Close the port and the files (torque as it is)."""
        self._arm.close()
        self._log.close()

    @property
    def pose(self) -> Pose | None:
        """The pose the arm was last sent to by a move along the tip (move_to, line_to, move_by, follow), or
        None. line_to and move_by go on from it; set it after a move_joints to a planned pose."""
        return self._arm.pose

    @pose.setter
    def pose(self, p: Pose | None) -> None:
        self._arm.pose = p

    @property
    def goal(self) -> dict:
        """The wanted angles last sent, by joint."""
        return self._arm.goal

    @property
    def torque(self) -> bool:
        return self._arm.torque

    @property
    def folded(self) -> bool:
        """The arm stood folded in its start pose at torque-on."""
        return self._arm.folded

    @property
    def start_pose(self) -> dict:
        """Where home() folds to: as the arm stood at torque-on if folded, else the fold from the ranges."""
        return self._arm.start_pose

    @property
    def step(self) -> str:
        """What the arm is doing, as the log and the CSV name it."""
        return self._arm.step

    @step.setter
    def step(self, text: str) -> None:
        self._arm.step = text

    @property
    def lead(self) -> tuple[float, ...]:
        """Degrees each of pan, lift, elbow and wrist flex is led ahead (follow(lead=True)); set 0 to drop it."""
        return self._arm.lead

    @lead.setter
    def lead(self, value) -> None:
        self._arm.lead = value

    @property
    def table_z(self) -> float:
        """The table's height (m) for the arm's own checks (Model.table_z's default)."""
        return self._arm.table_z

    @table_z.setter
    def table_z(self, z: float) -> None:
        self._arm.table_z = z

    @property
    def contact(self) -> bool:
        """The last follow with a press stopped on contact."""
        return self._arm.contact

    @property
    def offsets(self) -> tuple[float, ...]:
        """Degrees from calibrated to URDF angles, pan to roll."""
        return self._arm.offsets

    @property
    def raw(self):
        """The C library's arm: its settings (tick, sag, lag_max, load_max, offsets) and every call."""
        return self._arm
