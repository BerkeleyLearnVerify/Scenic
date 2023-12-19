import math

# NOTE: add your world info path here
# param worldInfoPath = "[YOUR PATH HERE]"

model scenic.simulators.airsim.model
from scenic.simulators.airsim.utils import getPrexistingObj
import random

ground = getPrexistingObj("ground")

ground.highlight()


ego = new StaticObj on ground,
    with assetName "Cone",
    with width 10,
    with length 10,
    with height 10


centerArea = RectangularRegion(Vector(0,200,30), 0, 100,100)

blocks = []
blockCount = 10


positions = []

for i in range(blockCount):
    pos = Vector(i*3,math.cos(i)*Uniform(1,3),1)
    positions.append(pos)
    blocks.append(new StaticObj at pos - Vector(0,0,1), 
        with assetName "Sphere",
        with width 1,
        with length 1,
        with height 1)
    

ranBlock = blocks[random.randint(0,blockCount-1)]


drone = new Drone on positions[0], with behavior Patrol(positions,smooth=True,speed=5)

