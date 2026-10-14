"""A triangle in the air with the follower's gripper: one move to a corner, then three relative moves.
Coordinates in metres: x forward, y to the arm's left, z up from the base.

Before it: the arm folded in its start pose and nothing within its reach (README, Safety)."""
from so101ik import Arm

with Arm(name="triangle") as arm:      # torque on and unfolded; folds home at the end, also after Ctrl-C
    arm.move_to(0.20, 0.12, 0.18)      # absolute: the first corner
    arm.move_by(0, 0.08, 0)            # relative: 8 cm to the left,
    arm.move_by(0, -0.04, 0.07)        # up to the top corner,
    arm.move_by(0, -0.04, -0.07)       # and back down to the first
