"""The kinematics alone, no arm needed: the least-loaded pose for a point, why a pose is refused, and
LeRobot's RobotKinematics calls answered by the C library."""
import numpy as np

from so101ik import Model, RobotKinematics, Unreachable

model = Model()                              # zero offsets and the calibrated ranges, or the built-in ones
pose = model.ik(0.20, 0.12, 0.18)            # the tip 20 cm forward, 12 cm left, 18 cm up
print("least-loaded pose:", pose)
print("joints (calibrated degrees):", {k: round(v, 1) for k, v in pose.deg.items()})

pose = model.ik(0.25, 0.0, 0.03, roll=-90)   # low over the table, the jaws turned
print("down at the table:", pose)

try:
    model.ik(0.60, 0.0, 0.10)                # out of reach
except Unreachable as e:
    print("unreachable:", e)

# The drop-in for lerobot.model.kinematics.RobotKinematics: URDF degrees in and out.
kin = RobotKinematics()
q = np.array([0.0, -30.0, 40.0, 60.0, 0.0, 0.0])
T = kin.forward_kinematics(q)
T[:3, 3] += [0.0, 0.02, 0.0]                 # the same frame 2 cm to the left
q2 = kin.inverse_kinematics(q, T)            # LeRobot's weights: position 1, orientation 0.01
print("inverse_kinematics:", q2.round(2), f"{kin.error * 1000:.3f} mm off after {kin.steps} steps")
q3 = kin.inverse_kinematics(q, T, orientation_weight=0)   # five joints cannot always hold the orientation too
print("position alone:    ", q3.round(2), f"{kin.error * 1000:.3f} mm off after {kin.steps} steps")
