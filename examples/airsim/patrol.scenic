# NOTE: add your world info path here
# param worldInfoPath = "[YOUR PATH HERE]"

model scenic.simulators.airsim.model


drone1 = new Drone at (0,0,3),
    with behavior Patrol([(-10,20,10),(10,40,10),(-10,40,10),(-10,20,10),(10,20,10),(10,40,10),(-10,40,10)],True)