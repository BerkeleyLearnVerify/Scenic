"""Place a USD asset on an existing table using the asset's own geometry as its shape.

The Rubik's cube is spawned from the Isaac asset library, but Scenic needs a
mesh to reason about placement and collisions. That mesh is produced from the
USD once with the conversion tool (see "Converting USD assets to meshes" in the
README):

    python src/scenic/simulators/isaac/usd_to_mesh.py \\
        --folders examples/isaacsim/assets/rubiks_cube --load-materials

after copying ``Isaac/Props/Rubiks_Cube/rubiks_cube.usd`` from a local Isaac
asset pack into ``examples/isaacsim/assets/rubiks_cube/``.
"""

import trimesh

param environmentUSDPath = "Isaac/Environments/Simple_Room/simple_room.usd"
param cubeSize = 0.2

from lib import *
model scenic.simulators.isaac.model
from scenic.simulators.isaac.utils import getExistingObj
from scenic.core.utils import repairMesh

table = getExistingObj("/Root/table_low_327/table_low")

class RubiksCube(IsaacSimObject):
    shape: MeshShape(repairMesh(trimesh.load(
        localPath("assets/rubiks_cube/_converted/rubiks_cube_usd.gltf")).to_geometry()))
    width: globalParameters.cubeSize
    length: globalParameters.cubeSize
    height: globalParameters.cubeSize
    isaacAssetPath: "Isaac/Props/Rubiks_Cube/rubiks_cube.usd"
    physics: False

cube = new RubiksCube on table

reference_toy = new Toy on table, right of cube by 0.35,
    with physics False
