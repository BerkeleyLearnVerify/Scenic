import math

# NOTE: add your world info path here
# param worldInfoPath = "[YOUR PATH HERE]"

model scenic.simulators.airsim.model

behavior sinTravel(speed = 5):
    client = simulation().client

    x = 0
    while True:
        x += 1
        do MoveByVelocity((0,0,math.sin(x)),1/speed)



drone1 = new Drone at (0,0,0), with behavior sinTravel()
   