# NOTE: add your world info path here
# param worldInfoPath = "[YOUR PATH HERE]"

model scenic.simulators.airsim.model


drone1 = new Drone at (0,0,0),
    with behavior Patrol([(-1,2,2),(1,4,2),(-1,4,2),(-1,2,4),(1,2,4),(1,4,4),(-1,4,4)],True)