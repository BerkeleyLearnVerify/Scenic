# NOTE: add your world info path here
# param worldInfoPath = "[YOUR PATH HERE]"
param worldInfoPath = "/home/msts/Testing/more/worldInfo"

model scenic.simulators.airsim.model

behavior Follow(target, speed = 5,tolerance = 5):
    client = simulation().client

    while True:
        targetPosition = target.position - (3,0,0)
        distance = (self.position - targetPosition).magnitude
        if distance < tolerance:
            do FlyToPosition(targetPosition, speed)
        wait



drone1 = new Drone at (0,0,0),
    with behavior Patrol([(-1,2,2),(1,4,2),(-1,4,2),(-1,2,4),(1,2,4),(1,4,4),(-1,4,4)],True)

drone2 = new Drone at (0,20,0),
    with behavior Follow(drone1, 5, 5)
