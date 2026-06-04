"""Explicit end-effector pick-and-place with the Franka Panda.

Unlike ``franka.scenic`` (which runs the backend's built-in pick-place state
machine via ``PickPlaceObject``), this example lays out each task step directly
as a Scenic behavior sequence.

Run with:
    scenic -S -b examples/isaacsim/robot/franka_move_to.scenic
"""

param isaacBackend = "experimental_60"
param environmentUSDPath = "Isaac/Environments/Simple_Room/simple_room.usd"
param duration = 60
param cubeSize = 0.0515
param binHeight = 0.1475
param hoverHeight = 0.3
param retractHeight = 0.2
param releaseGap = 0.02
param handToFingertips = 0.0877

model scenic.simulators.isaac.model
from scenic.simulators.isaac.utils import getExistingObj

table = getExistingObj("/Root/table_low_327/table_low")
CUBE_POSITION = (0.5, 0.3)
BIN_POSITION = (0.3, -0.3)

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

cube = new PickCube on table, at CUBE_POSITION

small_bin = new IsaacBin on table, at BIN_POSITION

place_pos = (small_bin.x, small_bin.y, cube.z)

def handTargetForFingertips(pos):
    return (pos[0], pos[1], pos[2] + globalParameters.handToFingertips)

behavior FrankaMoveToPickPlace(target_object, place_pos):
    pick_pos = (target_object.x, target_object.y, target_object.z)
    home = handTargetForFingertips((
        0.30,
        0.0,
        pick_pos[2] + globalParameters.hoverHeight,
    ))
    hover_pick = handTargetForFingertips((
        pick_pos[0],
        pick_pos[1],
        pick_pos[2] + globalParameters.hoverHeight,
    ))
    at_pick = handTargetForFingertips(pick_pos)

    release_pos = (
        place_pos[0],
        place_pos[1],
        place_pos[2] + globalParameters.releaseGap,
    )
    hover_place = handTargetForFingertips((
        place_pos[0],
        place_pos[1],
        place_pos[2] + globalParameters.retractHeight,
    ))
    at_place = handTargetForFingertips(release_pos)

    do MoveEndEffectorTo(home)
    do OpenGripper()
    do MoveEndEffectorTo(hover_pick)
    do MoveEndEffectorTo(at_pick)
    do HoldPosition()
    do CloseGripper()
    do MoveEndEffectorTo(hover_pick)
    do MoveEndEffectorTo(hover_place)
    do MoveEndEffectorTo(at_place)
    do OpenGripper()
    do MoveEndEffectorTo(hover_place)
    do MoveEndEffectorTo(home)
    terminate simulation

ego = new FrankaPanda on table, at (0, 0),
    with behavior FrankaMoveToPickPlace(
        cube,
        place_pos,
    )

print(f"CUBE POS: {cube.x}, {cube.y}, {cube.z}")
print(f"SMALL_BIN POS: {small_bin.x}, {small_bin.y}, {small_bin.z}")

require 0.42 <= distance from cube to ego <= 0.82
require 0.25 <= distance from small_bin to ego <= 0.82
require distance from cube to small_bin > 0.38

terminate after globalParameters.duration seconds
