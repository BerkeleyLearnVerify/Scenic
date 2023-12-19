import math

# NOTE: add your world info path here
# param worldInfoPath = "[YOUR PATH HERE]"

model scenic.simulators.airsim.model


def magnitude(v):
    return math.hypot(v.x, v.y, v.z)

behavior Follow(target, speed = 5,tolerance = 2, offset = (0,0,1)):
    client = simulation().client

    while True:
        targetPosition = target.position + offset
        
        velocity = targetPosition-self.position
        distance = magnitude(velocity)
        velocity = (velocity / distance) * speed
        if distance > tolerance:
            take MoveByVelocityUntilStopped(velocity)
        wait



drone1 = new Drone at (0,0,0),
    with behavior Patrol([(-1,2,2),(1,4,2),(-1,4,2),(-1,2,4),(1,2,4),(1,4,4),(-1,4,4)],True)

drone2 = new Drone at (0,1,0),
    with behavior Follow(drone1)
