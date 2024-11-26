import trimesh
import json
import os

from scenic.simulators.airsim.simulator import AirSimSimulator 
from scenic.simulators.airsim.actions import *
from scenic.simulators.airsim.behaviors import *
from scenic.simulators.airsim.utils import _addPrexistingObj, getPrexistingObj
from scenic.core.simulators import SimulationCreationError
from scenic.core.distributions import distributionFunction 

from scenic.simulators.airsim.utils import (
    airsimToScenicLocation,
    airsimToScenicOrientation,
    scenicToAirsimOrientation,
    scenicToAirsimScale,
    scenicToAirsimLocation,
)


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


    # scale_matrix = trimesh.transformations.scale_matrix([0, 0, -1])
    # tmesh.apply_transform(scale_matrix)
    
    center = (tmesh.bounds[0] + tmesh.bounds[1]) / 2
    print(center)
    
    
    # return MeshShape(tmesh)
    # todo other option to specify center
    blender_rot_conversion = (0, 90 deg, 0)
    return MeshShape(tmesh,initial_rotation=blender_rot_conversion, scale = .01)



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
    regionContainedIn: everywhere
    blueprint: "AirSimPrexisting"


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

class PX4Drone(Drone):
    blueprint: "PX4Drone"
    startHovering: True
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
        # convert unreal position to airsim position
        # TODO fix world info generator for airsim binaries
        position = Vector(meshData["position"][0],meshData["position"][1],meshData["position"][2])
        position = Vector(position.x,position.y,-position.z) # airsim uses -z as up instead of +z
        position *= 100 # airsim uses centimeters instead of meters

        rot = meshData["orientation"]
        newObj = new AirSimPrexisting with name meshData["name"],
            at airsimToScenicLocation(airsim.Vector3r(position.x, position.y, position.z)),
            facing airsimToScenicOrientation(airsim.to_quaternion(rot[0] , rot[1],rot[2] ))
        
        _addPrexistingObj(newObj)


# generate list of assets
assets = []
for filename in os.listdir(worldInfoPath+"/assets"):
    if filename.endswith(".obj"):
        # append the obj name without the .obj extension
        assets.append(filename[:-4])


param assets = assets