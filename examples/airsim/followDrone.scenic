import math

# NOTE: add your world info path here
# param worldInfoPath = "[YOUR PATH HERE]"

# Spawns 1 drone (ego) that patrols specific points, drone 2 follows ego

param timestep = .1

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
            take SetVelocity(velocity)
        wait

behavior Patrol(positions, loop=True, smooth = False, speed = 5,tolerance = 2):
    while True:
        for pos in positions:
            do FlyToPosition(pos,speed=speed,pidMode= not smooth,tolerance=tolerance)
           
        if not loop:
            return

ego = new Drone at (0,0,0),
    with behavior Patrol([(-1,2,2),(1,4,2),(-1,4,2),(-1,2,4),(1,2,4),(1,4,4),(-1,4,4)],True)

drone2 = new Drone at (0,1,0),
    with behavior Follow(ego)

terminate after 10 seconds

record final (distance to drone2) as dist