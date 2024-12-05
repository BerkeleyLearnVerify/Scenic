# NOTE: add your world info path here
param worldInfoPath = r"C:\Users\Mary\Documents\Code\Scenic\more\WoodsWorldInfo2"

model scenic.simulators.airsim.model
from scenic.simulators.airsim.utils import getPrexistingObj

# automatically place all objs on the ground in the workspace
# workspace = Workspace(RectangularRegion(Vector(0,0,30), 0, 100,100))

ground = getPrexistingObj("Ground")


# drone1 = new Drone at (0,0,0)

# point  = new Point on ground

tree = new StaticObj at (2,2,24.84665039),
    with assetName "tree", with name "tree2", with width 1, with length 1, with height 1