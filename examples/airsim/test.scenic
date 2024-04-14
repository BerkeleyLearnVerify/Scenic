# ------------------------------------------------------ PROBLEM 1 ------------------------------------------------------
""" import math

# NOTE: add your world info path here
param worldInfoPath = "C:/Users/piegu/Scenic/examples/airsim/worldInfo/droneBlocks"

# drone1 patrols to every location in points. ego spawns somehwere in the center area and tries to catch it before time ends

model scenic.simulators.airsim.model

behavior Catch(target, speed = 5,tolerance = 1, offset = (0,0,0)):
    try:
        while True:
            targetPosition = target.position + offset
            
            velocity = targetPosition-self.position
            distance = magnitude(velocity)
            velocity = (velocity / distance) * speed
            if distance > tolerance:
                take SetVelocity(velocity)
            wait
    interrupt when (distance from self to drone1) < 3:
        print("Caught drone1!")
        terminate

behavior Patrol(positions, loop=True, smooth = False, speed = 5,tolerance = 2):
    while True:
        pos = VerifaiOptions(positions)
        do FlyToPosition(pos,speed=speed,pidMode= not smooth,tolerance=tolerance)
           
        if not loop:
            return

centerArea = RectangularRegion(Vector(0,0,0), 0, 30, 30)

points = [(-1,2,2),(1,4,2),(-1,4,2),(-1,2,4),(1,2,4),(1,4,4),(-1,4,4)]

drone1 = new Drone at (0,0,0),
    with behavior Patrol(points,True)

ego = new Drone in centerArea,
    with behavior Catch(drone1)

terminate after 2 seconds """

# ------------------------------------------------------ PROBLEM 2 ------------------------------------------------------
""" import math

# NOTE: add your world info path here
param worldInfoPath = "C:/Users/piegu/Scenic/examples/airsim/worldInfo/droneBlocks"

model scenic.simulators.airsim.model

centerArea = RectangularRegion(Vector(0,0,0), 0, 30, 30)

ego = new Drone in centerArea

drone1 = new Drone in centerArea

terminate after 2 seconds """


# ------------------------------------------------------ PROBLEM 3 ------------------------------------------------------
# NOTE: add your world info path here
""" param worldInfoPath = "C:/Users/piegu/Scenic/examples/airsim/worldInfo/droneBlocks"

model scenic.simulators.airsim.model


centerArea = RectangularRegion(Vector(0,200,30), 0, 100,100)

ego = new Drone at (0,0,0) 

obj1 = new StaticObj in centerArea, 
        with assetName "Cone", # use * to pick a random asset in assets
        with width Range(3,10),
        with length Range(3,10),
        with height 10

terminate after 2 seconds """