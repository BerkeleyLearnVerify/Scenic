from scenic.simulators.isaac.actions import *
import numpy as np

# for the create3 wheeled robot
behavior KeepMoving():

    threshold = .01
    while True:
        if np.linalg.norm(self.speed) < threshold:
            for i in range(100):
                take applyController([-.2, 0])
            for i in range(50):
                take applyController([0, np.pi])
        else:
            take applyController([.2, 0])

# for the jetbot robot
behavior JetbotDrive():
    while True:
        take applyController([.2, 0])

# for the kaya robot
behavior DriveForward():
    while True:
        take applyController([-0.7, 0.0, 0.0])

behavior RandomMovement():
    i = 0
    while True:
        if i < 300:
            take applyController([0.4, 0.0, 0.0])   # forward
        elif i < 600:
            take applyController([-0.4, 0.0, 0.0])  # backward
        elif i < 900:
            take applyController([0.0, 0.2, 0.0])   # strafe one way
        elif i < 1200:
            take applyController([0.0, -0.2, 0.0])  # strafe the other way
        elif i < 1500:
            take applyController([0.0, 0.0, 0.2])   # rotate
        else:
            i = 0
            take applyController([0.0, 0.0, 0.0])

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

def _position_reached(agent, sim, target, threshold):
    ee_pos, _ = agent.get_ee_pose(sim)
    ee = np.array(ee_pos, dtype=float).flatten()[:3]
    return np.linalg.norm(ee - target) <= threshold

def _orientation_reached(agent, sim, target, threshold):
    _, ee_orientation = agent.get_ee_pose(sim)
    return _quaternion_distance(ee_orientation, target) <= threshold

def _gripper_reached(agent, sim, target, threshold):
    positions = np.array(agent.get_gripper_positions(sim), dtype=float)
    positions = positions.flatten()[:len(target)]
    return np.linalg.norm(positions - target) <= threshold

behavior _RepeatAction(action):
    while True:
        take action

behavior _StepRotate(orientation):
    sim = simulation()
    while True:
        ee_pos, _ = self.get_ee_pose(sim)
        take MoveToEEPoseAction(ee_pos, orientation)

behavior MoveEndEffectorTo(position, orientation=None, threshold=0.035, max_steps=600):
    sim = simulation()
    target = np.array(position, dtype=float).flatten()[:3]
    startTime = sim.currentTime
    try:
        do _RepeatAction(MoveToEEPoseAction(position, orientation))
    interrupt when _position_reached(self, sim, target, threshold):
        abort
    interrupt when sim.currentTime - startTime >= max_steps:
        abort
    if not _position_reached(self, sim, target, threshold):
        raise ManipulatorTimeout(f"MoveEndEffectorTo did not converge within {max_steps} steps")


behavior RotateEndEffectorTo(orientation, threshold=0.05, max_steps=600):
    sim = simulation()
    target = np.array(orientation, dtype=float).flatten()[:4]
    startTime = sim.currentTime
    try:
        do _StepRotate(orientation)
    interrupt when _orientation_reached(self, sim, target, threshold):
        abort
    interrupt when sim.currentTime - startTime >= max_steps:
        abort
    if not _orientation_reached(self, sim, target, threshold):
        raise ManipulatorTimeout(f"RotateEndEffectorTo did not converge within {max_steps} steps")


behavior OpenGripper(threshold=0.002, max_steps=100):
    sim = simulation()
    target = np.array(self.get_gripper_target_positions(True), dtype=float)
    startTime = sim.currentTime
    try:
        do _RepeatAction(OpenGripperAction())
    interrupt when _gripper_reached(self, sim, target, threshold) or sim.currentTime - startTime >= max_steps:
        abort


behavior CloseGripper(threshold=0.002, max_steps=100):
    sim = simulation()
    target = np.array(self.get_gripper_target_positions(False), dtype=float)
    startTime = sim.currentTime
    try:
        do _RepeatAction(CloseGripperAction())
    interrupt when _gripper_reached(self, sim, target, threshold) or sim.currentTime - startTime >= max_steps:
        abort


behavior HoldPosition(max_steps=30):
    for i in range(max_steps):
        take HoldPositionAction()
