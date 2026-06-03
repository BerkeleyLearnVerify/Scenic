param isaacBackend = "experimental_60"
param environmentUSDPath = "Isaac/Environments/Simple_Room/simple_room.usd"
param duration = 10
param cubeSize = 0.0515
param binHeight = 0.1475
param dropHeight = 0.2

param ikMethod = "damped-least-squares"
param ikScale = 1.0
param ikDamping = 0.05
param ikMinSingularValue = 1e-5

model scenic.simulators.isaac.model
from scenic.simulators.isaac.utils import getExistingObj
from franka_panda_robot import *

ik_method = globalParameters.ikMethod
ik_scale = globalParameters.ikScale
ik_damping = globalParameters.ikDamping
ik_min_singular_value = globalParameters.ikMinSingularValue
drop_height = globalParameters.dropHeight
duration = globalParameters.duration

if ik_method not in IK_METHODS:
    raise ValueError(
        f"invalid ikMethod {ik_method!r}; expected one of {sorted(IK_METHODS)}"
    )

table = getExistingObj("/Root/table_low_327/table_low")

class IsaacBin(IsaacSimObject):
    length: 0.3
    width: 0.2
    height: globalParameters.binHeight
    physics: False
    shape: BoxShape()
    isaac_asset_path: "Isaac/Props/KLT_Bin/small_KLT.usd"

class PickCube(IsaacSimObject):
    width: globalParameters.cubeSize
    length: globalParameters.cubeSize
    height: globalParameters.cubeSize
    mass: 0.05
    color: (1, 0, 0)
    shape: BoxShape()

cube = new PickCube on table

small_bin = new IsaacBin on table,
    facing toward cube

ego = new CustomFrankaPanda on table, at (0, 0),
    facing toward cube,
    with behavior FrankaPickPlaceWithIK(
        cube,
        Vector(small_bin.x, small_bin.y, small_bin.z + drop_height),
        ik_method,
        ik_scale,
        ik_damping,
        ik_min_singular_value,
    )

print(f"CUBE POS: {cube.x}, {cube.y}, {cube.z}")
print(f"SMALL_BIN POS: {small_bin.x}, {small_bin.y}, {small_bin.z}")
print(f"IK METHOD: {ik_method}")

require 0.42 <= distance from cube to ego <= 0.82
require 0.25 <= distance from small_bin to ego <= 0.82
require distance from cube to small_bin > 0.38

terminate after duration * 2 seconds
