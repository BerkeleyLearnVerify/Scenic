
import math

import lgsvl

import scenic.simulators as simulators
import scenic.simulators.lgsvl.utils as utils
from scenic.core.vectors import Vector

class LGSVLSimulator(simulators.Simulator):
    def __init__(self, lgsvl_scene, address='localhost', port=8181, alwaysReload=False):
        super().__init__()
        self.client = lgsvl.Simulator(address=address, port=port)
        if alwaysReload or self.client.current_scene != lgsvl_scene:
            self.client.load(scene=lgsvl_scene)

    def createSimulation(self, scene):
        return LGSVLSimulation(scene, self.client)

class LGSVLSimulation(simulators.Simulation):
    def __init__(self, scene, client):
        super().__init__(scene)
        self.client = client
        self.timeStep = scene.params.get('time_step', 1.0/30)
        self.objects = scene.objects

        # Reset simulator (deletes all existing objects)
        self.client.reset()

        # Create LGSVL objects corresponding to Scenic objects
        self.lgsvlObjects = {}
        for obj in self.objects:
            # Figure out what type of LGSVL object this is
            if not hasattr(obj, 'lgsvlObject'):
                continue    # not an LGSVL object
            if not hasattr(obj, 'lgsvlName'):
                raise RuntimeError(f'object {obj} does not have an lgsvlName property')
            if not hasattr(obj, 'lgsvlAgentType'):
                raise RuntimeError(f'object {obj} does not have an lgsvlAgentType property')
            name = obj.lgsvlName
            agentType = obj.lgsvlAgentType

            # Set up position and orientation
            state = lgsvl.AgentState()
            if obj.elevation is None:
                obj.elevation = self.groundElevationAt(obj.position)
            state.transform.position = utils.scenicToLGSVLPosition(obj.position, obj.elevation)
            state.transform.rotation = utils.scenicToLGSVLRotation(obj.heading)

            # Create LGSVL object
            lgsvlObj = self.client.add_agent(name, agentType, state)
            obj.lgsvlObject = lgsvlObj

        # TODO reset object controllers???

    def groundElevationAt(self, pos):
        origin = utils.scenicToLGSVLPosition(pos, 100000)
        result = self.client.raycast(origin, lgsvl.Vector(0, -1, 0))
        if result is None:
            print(f'WARNING: no ground at position {pos}')
            return 0
        return result.point.y

    def writePropertiesToLGSVL(self):
        for obj in self.objects:
            lgsvlObj = obj.lgsvlObject
            state = lgsvlObj.state
            # position, elevation
            state.transform.position = utils.scenicToLGSVLPosition(obj.position, y=obj.elevation)
            # heading
            state.transform.rotation = utils.scenicToLGSVLRotation(obj.heading)
            # update state
            lgsvlObj.state = state

    def readPropertiesFromLGSVL(self):
        for obj in self.objects:
            lgsvlObj = obj.lgsvlObject
            state = lgsvlObj.state
            # position, elevation
            obj.position = utils.lgsvlToScenicPosition(state.position)
            obj.elevation = utils.lgsvlToScenicElevation(state.position)
            # heading
            heading = utils.lgsvlToScenicRotation(state.rotation, tolerance2D=5)
            if heading is None:
                raise RuntimeError(f'{lgsvlObj} has non-planar orientation!')
            obj.heading = heading

    def currentState(self):
        return tuple(obj.position for obj in self.objects)

    def initialState(self):
        return self.currentState()

    def step(self, actions):
        # execute actions
        for agent, action in actions.items():
            if action is not None:
                action.applyTo(agent, agent.lgsvlObject, self)
        # run simulation for one time step
        self.client.run(time_limit=self.timeStep)
        # read back the results of the simulation
        self.readPropertiesFromLGSVL()
        return self.currentState()

class MoveAction(simulators.Action):
    def __init__(self, offset):
        self.offset = offset

    def applyTo(self, obj, lgsvlObject, sim):
        pos = obj.position.offsetRotated(obj.heading, self.offset)
        pos = utils.scenicToLGSVLPosition(pos, y=obj.elevation)
        state = lgsvlObject.state
        state.transform.position = pos
        lgsvlObject.state = state

class SetVelocityAction(simulators.Action):
    def __init__(self, velocity):
        self.velocity = utils.scenicToLGSVLPosition(velocity)

    def applyTo(self, obj, lgsvlObject, sim):
        state = lgsvlObject.state
        state.velocity = self.velocity
        lgsvlObject.state = state

class FollowWaypointsAction(simulators.Action):
    def __init__(self, waypoints):
        self.waypoints = tuple(waypoints)
        self.lastTime = -2

    def applyTo(self, obj, lgsvlObject, sim):
        #print(sim.currentTime, self.lastTime)
        if sim.currentTime is not self.lastTime + 1:
            lgsvlObject.follow(self.waypoints)
        self.lastTime = sim.currentTime

class CancelWaypointsAction(simulators.Action):
    def applyTo(self, obj, lgsvlObject, sim):
        lgsvlObject.walk_randomly(False)
