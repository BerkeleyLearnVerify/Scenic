from scenic.simulators.isaac.actions import *
import numpy as np

# for the create3 wheeled robot
behavior KeepMoving():

    threshold = .01
    while True:
        if np.linalg.norm(self.speed) < threshold:
            for i in range(100):
                take applyWheeledController(-.2, 0)
            for i in range(50):
                take applyWheeledController(0, np.pi)
        else:
            take applyWheeledController(.2, 0)

# for the jetbot robot
behavior JetbotDrive():
    while True:
        take applyController([.2, 0])

# for the kaya robot
behavior DriveForward():
    while True:
        take applyHolonomicController(-0.7, 0.0, 0.0)

behavior RandomMovement():
    i = 0
    while True:
        if i >= 0 and i < 500:
            take applyHolonomicController(-0.7, 0.0, 0.0)
        elif i >= 500 and i < 1000:
            take applyHolonomicController(0.0, 0.4, 0.0)
        elif i >= 1000 and i < 1100:
            take applyHolonomicController(0.0, 0.0, 0.05)
        elif i == 1200:
            i = 0
        i += 1

# for the Franka Panda manipulator
behavior PickPlaceObject(target_object, goal_position):
    while True:
        take applyPickPlaceController(target_object, goal_position)


# Generic manipulator primitives. Scenario-level behaviors should compose these
# with `do ...` so the overall task is visible in the scenario file.

def _unit_quaternion(quaternion):
    quaternion = np.array(quaternion, dtype=float).flatten()[:4]
    return quaternion / np.linalg.norm(quaternion)

def _quaternion_distance(quaternion_a, quaternion_b):
    quaternion_a = _unit_quaternion(quaternion_a)
    quaternion_b = _unit_quaternion(quaternion_b)
    dot = abs(float(np.dot(quaternion_a, quaternion_b)))
    return 2 * np.arccos(np.clip(dot, -1.0, 1.0))

behavior MoveEndEffectorTo(position, orientation=None, threshold=0.035,
                           max_steps=None):
    sim = simulation()
    target = np.array(position, dtype=float).flatten()[:3]
    steps = 0
    while True:
        take SetEEPoseAction(position, orientation)
        steps += 1
        ee_pos, _ = self.get_ee_pose(sim)
        ee = np.array(ee_pos, dtype=float).flatten()[:3]
        dist = np.linalg.norm(ee - target)
        if dist <= threshold:
            break
        if max_steps is not None and steps >= max_steps:
            break


behavior RotateEndEffectorTo(orientation, threshold=0.05, max_steps=None):
    sim = simulation()
    target = np.array(orientation, dtype=float).flatten()[:4]
    steps = 0
    while True:
        ee_pos, _ = self.get_ee_pose(sim)
        take SetEEPoseAction(ee_pos, target)
        steps += 1
        _, ee_orientation = self.get_ee_pose(sim)
        if _quaternion_distance(ee_orientation, target) <= threshold:
            break
        if max_steps is not None and steps >= max_steps:
            break


behavior OpenGripper(threshold=0.002, max_steps=100):
    sim = simulation()
    target = np.array(self.get_gripper_target_positions(True), dtype=float)
    steps = 0
    while True:
        take OpenGripperAction()
        steps += 1
        positions = np.array(self.get_gripper_positions(sim), dtype=float)
        positions = positions.flatten()[:len(target)]
        if np.linalg.norm(positions - target) <= threshold:
            break
        if max_steps is not None and steps >= max_steps:
            break


behavior CloseGripper(threshold=0.002, max_steps=100):
    sim = simulation()
    target = np.array(self.get_gripper_target_positions(False), dtype=float)
    steps = 0
    while True:
        take CloseGripperAction()
        steps += 1
        positions = np.array(self.get_gripper_positions(sim), dtype=float)
        positions = positions.flatten()[:len(target)]
        if np.linalg.norm(positions - target) <= threshold:
            break
        if max_steps is not None and steps >= max_steps:
            break


behavior HoldPosition(max_steps=30):
    for i in range(max_steps):
        take HoldPositionAction()
