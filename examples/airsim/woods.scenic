# NOTE: add your world info path here
param worldInfoPath = r"C:\Users\Mary\Documents\Code\Scenic\more\WoodsWorldInfo2"

model scenic.simulators.airsim.model
from scenic.simulators.airsim.utils import getPrexistingObj

# automatically place all objs on the ground in the workspace
# workspace = Workspace(RectangularRegion(Vector(0,0,30), 0, 100,100))

ground = getPrexistingObj("Ground")
orangeBall = getPrexistingObj("OrangeBall")


floor = new StaticObj with assetName "Cube", with width 1000, with length 1000, with height 0.01, with color [.5,.5,.5], at (0,0,0)

# drone1 = new Drone at (0,0,0)


# tree = new StaticObj at (2,2,24.84665039),
#     with assetName "tree", with name "tree2"

# cone = new StaticObj at (2,2,24.84665039),
#     with assetName "Cone",   facing (0, 90 deg, 0)
    
# cone2 = new StaticObj at (5,2,24.84665039),
#     with assetName "Cone",  facing (0, 0, 90 deg)

# cone3 = new StaticObj on cone2,
#     with assetName "Cone",  facing (0, 0, 90 deg)


regionSize = 100
for i in range(2):
    for j in range(2):
        # make chunk area at point contained in floor region
        chunkArea = RectangularRegion(Vector(i*regionSize,j*regionSize,0), 0, regionSize,regionSize)
        # chunkAreas.append(chunkArea)
    

        tree = new StaticObj on floor,
            contained in chunkArea,
            with assetName "tree1",

        # print(tree.width,tree.length)
        # for i in range(2):
        #     bush = new StaticObj with assetName "bush1", on floor, contained in chunkArea
