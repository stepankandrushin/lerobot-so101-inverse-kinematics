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

    SO101_PORT          the follower's serial port, e.g. /dev/tty.usbmodem58760431541 or /dev/ttyACM0
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
    """The follower on its servo bus, for a with block: torque on and unfolded from the start pose at the
    start, folded home and torque off at the end, also after an error. Ctrl-C (or a kill, a hang-up)
    stops the move and the arm goes home; a second one switches the torque off, and the arm drops.

    port and calibration_file default to find_port() and find_calibration(), offsets to SO101_OFFSETS (see
    read_offsets). Every tick goes to OUT/NAME.csv, every event to OUT/NAME.log and stdout. Before a run:
    the arm folded in its start pose, nothing in its way."""

    def __init__(self, port: str | None = None, calibration_file: Path | str | None = None, offsets=None,
                 name: str | None = None, out: Path | str = Path("outputs") / "so101ik"):
        calibration_file = Path(calibration_file) if calibration_file else find_calibration()
        cal = read_calibration(calibration_file)
        offsets = read_offsets(offsets)
        port = port or find_port(cal)
        out = Path(out)
        out.mkdir(parents=True, exist_ok=True)
        self.name = name or time.strftime("arm-%Y%m%d-%H%M%S")
        self._log = (out / f"{self.name}.log").open("a")
        self._arm = _Arm(port, cal, log=self._line, csv=str(out / f"{self.name}.csv"))
        self._arm.offsets = offsets
        self._signals = {}
        self._arm.log(f"port {port}, calibration {calibration_file}, offsets "
                      + " ".join(f"{v:+g}" for v in offsets))

    def _line(self, line: str) -> None:
        self._log.write(line + "\n")
        self._log.flush()
        try:
            print(line, flush=True)
        except OSError:
            pass

    def _stop(self, *_):
        raise KeyboardInterrupt

    def __enter__(self) -> "Arm":
        try:
            self._arm.check()
            for sig in (signal.SIGTERM, signal.SIGHUP):
                self._signals[sig] = signal.signal(sig, self._stop)
            self._arm.torque_on()
            if self._arm.folded:
                self._arm.unfold()
        except BaseException:
            self.__exit__(None, None, None)
            raise
        return self

    def __exit__(self, kind, value, tb) -> bool:
        try:
            if self._arm.torque:
                if kind is not None:
                    self._arm.log(f"{kind.__name__}: {value}; going home")
                self._arm.home()
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

    def move_to(self, x: float, y: float, z: float, pitch: float | None = None, speed: float = 30.0) -> Pose:
        """The tip to (x, y, z) m, in the least-loaded allowed pose (or at pitch, degrees), straight in joint
        space at a peak of speed degrees/s."""
        return self._arm.move_to(x, y, z, pitch, speed)

    def move_by(self, dx: float, dy: float, dz: float, speed: float = 0.05) -> Pose:
        """The tip dx, dy, dz m from where it is, along a straight line at a peak of speed m/s."""
        return self._arm.move_by(dx, dy, dz, speed)

    def line_to(self, x: float, y: float, z: float, speed: float = 0.05) -> Pose:
        """The tip along a straight line to (x, y, z) m at a peak of speed m/s."""
        return self._arm.line_to(x, y, z, speed)

    def move_joints(self, target, speed: float = 30.0, check: bool = True) -> None:
        """The joints in target (a dict by name, or all six) to their angles, straight in joint space."""
        self._arm.move_joints(target, speed, check)

    def home(self) -> None:
        self._arm.home()

    def read(self):
        """(degrees, load %, mA), each a dict by joint."""
        return self._arm.read()

    @property
    def pose(self) -> Pose | None:
        return self._arm.pose

    @property
    def offsets(self) -> tuple[float, ...]:
        """Degrees from calibrated to URDF angles, pan to roll."""
        return self._arm.offsets

    @property
    def raw(self):
        """The C library's arm: its settings (tick, sag, lag_max, load_max, offsets) and every call."""
        return self._arm
