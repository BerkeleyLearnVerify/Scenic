# NOTE: add your world info path here
# param worldInfoPath = "[YOUR PATH HERE]"

model scenic.simulators.airsim.model


platforms = []
ground = getPrexistingObj("Ground")
centerArea = RectangularRegion(Vector(0,0,30), 0, 100,100)


blockCount = 10
for i in range(blockCount):
    platforms.append(new StaticObj on ground, 
        contained in centerArea,
        with assetName "Cone", # use * to pick a random asset in assets
        with width Range(3,10),
        with length Range(3,10),
        with height 10)





points = []
for plat in platforms:
    platRegion = plat.occupiedSpace.boundingPolygon
    point = new Point on platRegion
    points.append(new Point at (point.position +Vector(0,0,15)))


# for point in points:
#     cube = new StaticObj with assetName "Cube", with width 1, with length 1, with height 1,with color [1,0,0], on point.position


drone1 = new Drone at Uniform(*points) + (0,0,5),
    with behavior Patrol(points,True)