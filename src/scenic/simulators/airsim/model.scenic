import trimesh
import json

from scenic.simulators.airsim.simulator import AirSimSimulator 
from scenic.simulators.airsim.actions import *
from scenic.simulators.airsim.behaviors import *
from scenic.simulators.airsim.utils import _addPrexistingObj, getPrexistingObj


# ---------- global parameters ----------

# setting Default global parameters for any missing parameters
param timestep = 1
param airsimWorldInfoPth = None
param idleStoragePos = (1000,1000,1000)
param worldInfoPath = "/home/mary/Documents/Scenic/src/scenic/simulators/airsim/objs/cubes/"

# ---------- helper functions ----------

def createMeshShape(subFolder, assetName):
    tmesh = trimesh.load( globalParameters.worldInfoPath+subFolder+"/"+assetName+".obj")
    


    return MeshShape(tmesh ,scale=.01)


# ---------- simulator creation ----------
simulator AirSimSimulator(timestep=globalParameters.timestep,idleStoragePos=globalParameters.idleStoragePos) 


# ---------- base classes ----------
class AirSimPrexisting:
    name: None
    shape: createMeshShape("objectMeshes",self.name)
    allowCollisions: True
    blueprint: "AirSimPrexisting"

    def highlight(self):
        tmesh = self.shape.mesh
        
        color = trimesh.visual.random_color()
        for vert in tmesh.visual.vertex_colors:
            vert[0] = color[0]
            vert[1] = color[1]
            vert[2] = color[2]
        
        self.shape = MeshShape(tmesh ,scale=.01)

class AirSimActor:
        
    name: None
    assetName: None
    blueprint: None
    realObjName: None 

    # override
    shape: createMeshShape("assets",self.assetName)

    def __str__(self):
        return self.assetName


# ---------- inherited classes ----------
class Drone(AirSimActor):
    blueprint: "Drone"
    startHovering: True
    assetName: "Quadrotor1"
    _startPos: None


class StaticObj(AirSimActor):
    blueprint: "StaticObj"
    physEnabled: False



# ---------- body ----------
# Create prexisiting airsim objs
prexisitingObjs = {}
with open(
    globalParameters.worldInfoPath+"worldInfo.json",
    "r",
) as inFile:
    # todo raise warning if there isn't a volume
    meshDatas = json.load(inFile)
    verbosePrint("\n\nPrexisting Object Names:\n",[md["name"] for md in meshDatas], level=1)
    for meshData in meshDatas:
        newObj = new AirSimPrexisting with name meshData["name"],
            at meshData["position"],
            facing meshData["orientation"]
        
        _addPrexistingObj(newObj)

