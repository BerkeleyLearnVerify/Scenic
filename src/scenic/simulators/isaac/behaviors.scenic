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
