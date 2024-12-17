# NOTE: add your world info path here
# param worldInfoPath = "[YOUR PATH HERE]"
param worldOffset = Vector(0,0,50) # blocks world offset


model scenic.simulators.airsim.model




# get the ground
ground = getPrexistingObj("Ground")

# predefine a center area
centerArea = RectangularRegion(Vector(0,0,30), 0, 100,100)

# spawn 10 cones in the center area ontop of the ground 
coneCount = 10
platforms = []
for i in range(coneCount):
    platforms.append(new StaticObj on ground, 
        contained in centerArea,
        with assetName "Cone", # use * to pick a random asset in assets
        with width Range(3,10),
        with length Range(3,10),
        with height 10)

points = []
for plat in platforms:
    # get the area ontop of each cone
    platRegion = plat.occupiedSpace.boundingPolygon

    # make a point directly above the region
    point = new Point on platRegion
    points.append(new Point at (point.position +Vector(0,0,15)))

# start drone at a random point and make it patrol between the points
drone1 = new Drone at Uniform(*points) + (0,0,5),
    with behavior Patrol(points,True)


