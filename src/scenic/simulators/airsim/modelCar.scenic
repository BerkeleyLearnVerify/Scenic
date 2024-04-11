import trimesh
import json
import os

from scenic.simulators.airsim.simulatorCar import AirSimSimulator 
from scenic.simulators.airsim.actions import *
from scenic.simulators.airsim.behaviors import *
from scenic.simulators.airsim.utils import _addPrexistingObj, getPrexistingObj
from scenic.core.simulators import SimulationCreationError
from scenic.core.distributions import distributionFunction 
from scenic.core.utils import repairMesh

# ---------- global parameters ----------

# setting Default global parameters for any missing parameters
param timestep = 1
param airsimWorldInfoPth = None
param idleStoragePos = (1000,1000,1000)
param worldInfoPath = ""
worldInfoPath = globalParameters.worldInfoPath


# ---------- helper functions ----------

@distributionFunction 
def printFinal(val):
    print(val)

@distributionFunction 
def createMeshShape(subFolder, assetName):
    objFile = assetName+".obj"
    tmesh = trimesh.load(worldInfoPath+subFolder+"/"+objFile)

    return MeshShape(tmesh ,scale=.01)

# def createMeshShape2(subFolder, assetName):
#     objFile = assetName+".obj"
#     tmesh = trimesh.load(worldInfoPath+subFolder+"/"+objFile)

#     tmesh = repairMesh(tmesh)

#     with open(
#         r"C:\Users\piegu\Scenic\examples\airsim" + r"\assets" + assetName + ".obj",
#         "w",
#     ) as outfile:
#         outfile.write(trimesh.exchange.obj.export_obj(tmesh))

#     return MeshShape(tmesh ,scale=.01)


# ---------- simulator creation ----------
simulator AirSimSimulator(timestep=globalParameters.timestep,idleStoragePos=globalParameters.idleStoragePos) 


# ---------- simulator getter funcs ----------

def getAssetNames():
    return self.client.simListAssets()

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
class Car(AirSimActor):
    blueprint: "Car"
    assetName: "Suv"
    _startPos: None

class StaticObj(AirSimActor):
    blueprint: "StaticObj"
    physEnabled: False

# ---------- body ----------



# ensure worldInfoPath is set
if not worldInfoPath:
    raise SimulationCreationError('\nworldInfoPath not set, use:\n param worldInfoPath = "[YOUR PATH HERE]" \n\n more information on creating and using worldInfo in docs')
else:
    worldInfoPath += "/"
    

# Create prexisiting airsim objs
prexisitingObjs = {}
with open(
    worldInfoPath+"worldInfo.json",
    "r",
) as inFile:
    meshDatas = json.load(inFile)
    for meshData in meshDatas:
        newObj = new AirSimPrexisting with name meshData["name"],
            at meshData["position"],
            facing meshData["orientation"]
        
        _addPrexistingObj(newObj)


# generate list of assets
assets = []
for filename in os.listdir(worldInfoPath+"/assets"):
    if filename.endswith(".obj"):
        # append the obj name without the .obj extension
        assets.append(filename[:-4])


param assets = assets