# NOTE: add your world info path here
# param worldInfoPath = "[YOUR PATH HERE]"
param worldInfoPath = "/home/msts/Testing/more/worldInfo"

model scenic.simulators.airsim.model

blocks = []
for i in range(blockCount):
    blocks.append(new StaticObj on ground, 
        contained in centerArea,
        with assetName Uniform
        with width Range(3,10),
        with length Range(3,10),
        with height 10)

print(assets)
# drone1 = new Drone at (0,0,0),