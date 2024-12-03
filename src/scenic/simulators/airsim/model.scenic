import trimesh
import json
import os
import numpy as np
import scipy

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


    center = (tmesh.bounds[0] + tmesh.bounds[1]) / 2
    print("center=",center)

    # scale_matrix = trimesh.transformations.scale_matrix([0, 0, -1])
    # tmesh.apply_transform(scale_matrix)
    
    
    # print(center)
    
    
    # return MeshShape(tmesh)

    scale_matrix = trimesh.transformations.compose_matrix(scale=(.01,.01,.01))
    tmesh.apply_transform(scale_matrix)

    rotation_matrix = trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0])
    tmesh.apply_transform(rotation_matrix)


    center = (tmesh.bounds[0] + tmesh.bounds[1]) / 2
    print("NEW center=",center)

    return MeshShape(tmesh), center




# ---------- simulator creation ----------
simulator AirSimSimulator(timestep=globalParameters.timestep,idleStoragePos=globalParameters.idleStoragePos) 


# ---------- simulator getter funcs ----------

def getAssetNames():
    return self.client.simListAssets()

# ---------- base classes ----------
# TODO OFFSETS
class AirSimPrexisting:
    name: None
    shape: None
    allowCollisions: True
    regionContainedIn: everywhere
    blueprint: "AirSimPrexisting"
    color: [.3,0,0] if self.name == "Ground" else [0.5, 0.5, 0.5]


class AirSimActor:
        
    name: None
    assetName: None
    blueprint: None
    realObjName: None 

    # override
    shape: createMeshShape("assets",self.assetName)[0]

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

        actorName = meshData["name"]
        print("\n\nname = ",actorName)

        meshShape,center = createMeshShape("objectMeshes",actorName)

        # convert unreal position to airsim position
        # TODO fix world info generator for airsim binaries
        position = Vector(meshData["position"][0],meshData["position"][1],meshData["position"][2])
        position += center*10000

        rot = meshData["orientation"]
        pitch,roll,yaw = rot[0] , rot[1],rot[2] 
        
        angles = (-yaw - 90, pitch,roll)
        r = scipy.spatial.transform.Rotation.from_euler(
            seq="ZXY", angles=angles, degrees=True
        )

    
        orientation =  Orientation(r)



        newObj = new AirSimPrexisting with shape meshShape, with name actorName,
            at airsimToScenicLocation(airsim.Vector3r(position.x, position.y, position.z)),
            facing orientation # pitch, roll, yaw
     

        _addPrexistingObj(newObj)


# generate list of assets
assets = []
for filename in os.listdir(worldInfoPath+"/assets"):
    if filename.endswith(".obj"):
        # append the obj name without the .obj extension
        assets.append(filename[:-4])


param assets = assets