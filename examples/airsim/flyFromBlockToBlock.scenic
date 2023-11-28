# NOTE: add your world info path here
# param worldInfoPath = "[YOUR PATH HERE]"
param worldInfoPath = "/home/msts/Testing/more/worldInfo"

model scenic.simulators.airsim.model


platforms = []

blockCount = 10
for i in range(blockCount):
    objs.append(new StaticObj on ground, 
        contained in centerArea,
        with assetName Uniform(*assets), # use * to pick a random asset in assets
        with width Range(3,10),
        with length Range(3,10),
        with height 10)





points = []
for plat in platforms:
    point = new Point on plat

drone1 = new Drone on Uniform(*platforms) with behavior Patrol(points)