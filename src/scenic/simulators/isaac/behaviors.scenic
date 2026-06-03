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
