import trimesh

param environmentUSDPath = localPath("../../assets/Simple_Room/simple_room.usd")

param testUSDPath = localPath("../../assets/Rubiks_Cube/rubiks_cube.usd")
param testAssetWidth = 0.17
param testAssetLength = 0.17
param testAssetHeight = 0.17

from lib import *
model scenic.simulators.isaac.model
from scenic.simulators.isaac.utils import getExistingObj

simple_room_table = getExistingObj("/Root/table_low_327/table_low")

class IsaacAssetBox(IsaacSimObject):
    shape: MeshShape(repairMesh(trimesh.load(localPath("../../assets/Rubiks_Cube/_converted/rubiks_cube_usd.gltf")).to_geometry()))
    # width: globalParameters.testAssetWidth
    # length: globalParameters.testAssetLength
    # height: globalParameters.testAssetHeight
    # usd_path: globalParameters.testUSDPath
    isaac_asset_path: "Isaac/Props/Rubiks_Cube/rubiks_cube.usd"
    physics: True

test_asset = new IsaacAssetBox above simple_room_table by 0.5

reference_toy = new Toy right of test_asset by 0.35,
    with physics False
