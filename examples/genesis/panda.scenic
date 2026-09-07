"""RL Grasp Training/Evaluation Scenario for Genesis."""

# Training configuration parameters
param max_iterations = 400
param max_steps = 300

param workspaceDim = 4
param meshesPath = localPath("../../assets/meshes")

model scenic.simulators.genesis.model
import numpy as np

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
    height: 0.2 # TODO: Random scales?
    shape: MeshShape.fromFile(globalParameters.meshesPath / "dining_table.obj.bz2")
    fixed: True
    collision: True

# Define floor and workspace
floor = new Floor at (0,0,0),
    with width globalParameters.workspaceDim, with length globalParameters.workspaceDim
workspace = Workspace(RectangularRegion((0,0), 0, globalParameters.workspaceDim + 0.01, globalParameters.workspaceDim + 0.01))

# Define robot
class GymPandaRobotArm(PandaRobotArm):
    def genesisStartDynamicSimulation(self):
        self.genesis_entity.set_dofs_kp(
            np.array([4500, 4500, 3500, 3500, 2000, 2000, 2000, 100, 100]),
        )
        self.genesis_entity.set_dofs_kv(
            np.array([450, 450, 350, 350, 200, 200, 200, 10, 10]),
        )
        self.genesis_entity.set_dofs_force_range(
            np.array([-87, -87, -87, -87, -12, -12, -12, -100, -100]),
            np.array([87, 87, 87, 87, 12, 12, 12, 100, 100]),
        )

    def executeActions(self, actions):
        self.genesis_entity.control_dofs_position(actions)

ego = new GymPandaRobotArm at (0,0), on floor, with name "Panda"

table = new Table ahead of ego by 0.25, on floor
new Box on table, with color (1,0,0), with name "Target"

# require distance from box to ego > 0.4