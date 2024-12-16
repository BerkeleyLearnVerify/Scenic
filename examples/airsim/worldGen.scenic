# NOTE: add your world info path here
# param worldInfoPath = "[YOUR PATH HERE]"
param worldOffset = Vector(0,0,50) # blocks world offset

model scenic.simulators.airsim.model
from scenic.simulators.airsim.utils import getPrexistingObj
import random

ground = getPrexistingObj("Ground")


ego = new StaticObj on ground,
    with assetName "Cone",
    with width 10,
    with length 10,
    with height 10


centerArea = RectangularRegion(Vector(0,200,30), 0, 100,100)

blocks = []
blockCount = 10



for i in range(blockCount):
    blocks.append(new StaticObj on ground, 
        contained in centerArea,
        with assetName "Cube",
        with width Range(3,10),
        with length Range(3,10),
        with height 10)
    

ranBlock = Uniform(*blocks)

drone = new Drone on ranBlock, with behavior Patrol([(10,10,10),(2,2,2)])