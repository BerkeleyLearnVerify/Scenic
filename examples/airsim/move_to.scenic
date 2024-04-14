# NOTE: add your world info path here
# param worldInfoPath = "[YOUR PATH HERE]"

model scenic.simulators.airsim.model


ego = new Drone at (0,0,0),
    with behavior MoveToPosition((0,10,10),5)
    