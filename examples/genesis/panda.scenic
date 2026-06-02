"""RL Grasp Training/Evaluation Scenario for Genesis."""

# Training configuration parameters
param max_iterations = 400
param max_steps = 300

param workspaceDim = 4
param meshesPath = localPath("../../assets/meshes")

model scenic.simulators.genesis.model

# Scenario Classes
class Floor(GenesisMeshObject):
    shape: BoxShape()
    height: 0.01
    fixed: True
    collision: True

class Box(GenesisMeshObject):
    width: 0.1
    length: 0.1
    height: 0.1
    shape: BoxShape()
    collision: True

class Ball(GenesisMeshObject):
    width: 0.1
    length: 0.1
    height: 0.1
    shape: SpheroidShape()
    collision: True

class Table(GenesisMeshObject):
    width: 1
    length: 0.5
    height: Range(0.2, 0.3) # TODO: Random scales?
    shape: MeshShape.fromFile(globalParameters.meshesPath / "dining_table.obj.bz2")
    fixed: True
    collision: True

# Define floor and workspace
floor = new Floor at (0,0,0),
    with width globalParameters.workspaceDim, with length globalParameters.workspaceDim
workspace = Workspace(RectangularRegion((0,0), 0, globalParameters.workspaceDim + 0.01, globalParameters.workspaceDim + 0.01))

# Define robot
robot = new PandaRobotArm at (0,0), on floor

table = new Table ahead of robot by 0.25, on floor
new Box on table, with color (1,0,0)

# require distance from box to robot > 0.4