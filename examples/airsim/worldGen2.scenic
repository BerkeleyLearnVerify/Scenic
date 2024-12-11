# NOTE: add your world info path here
param worldInfoPath = r"C:\Users\Mary\Documents\Code\Scenic\more\worldInfo"

model scenic.simulators.airsim.model
from scenic.simulators.airsim.utils import getPrexistingObj
import random

ground = getPrexistingObj("Ground")

centerArea = RectangularRegion(Vector(0,200,30), 0, 100,100)

buildings = []
platforms = []
blockCount = 10



for i in range(blockCount):
    building = new StaticObj on ground, 
        contained in centerArea,
        with assetName "Cube",
        with width Range(3,10),
        with length Range(3,10),
        with height 10

    buildings.append(building)
    # create a block with
    

    # attach another block to the side of it to act as the balcony base
    
    balconyFloor = new StaticObj  left of building,
        # contained in centerArea,
        with assetName "Cube",
        
        
        

    fence = new StaticObj  left of balconyFloor,
        # contained in centerArea,
        with assetName "Cube",
        with width .1,
        with length 4,
        with height 3

ranBlock = Uniform(*buildings)

drone = new Drone on ranBlock