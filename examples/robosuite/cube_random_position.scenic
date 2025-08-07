"""Randomized cube position using behavior library."""

model scenic.simulators.robosuite.model

param use_environment = "Lift"
param render = True
param real_time = False
param camera_view = "birdview"
# press escape in simulater inrterface to reset to robosuite default frontview,for seeing the cube actually being lifted. 
# press ']' or '[' to cycle between different fixed camera_views. (pre defined/set in robosuits's task enviroments)
# other options for camera_view:
    # "frontview"
    # "birdview"
    # "agentview"
    # "sideview"
    # "robot0_robotview"
    # "robot0_eye_in_hand"


# Randomize cube position
cube = new LiftCube at (Range(-0.25, 0.25), Range(-0.25, 0.25), 0.82)

# Clean approach using object reference
ego = new PandaRobot with behavior PickAndLift(cube, height=1.05)

# other robot options: UR5eRobot

# Alternative: Custom composition
# behavior CustomLift():
#     do PickObject(cube)
#     do LiftToHeight(1.05)
#     if simulation().checkSuccess():
#         terminate simulation

# ego = new PandaRobot with behavior CustomLift()