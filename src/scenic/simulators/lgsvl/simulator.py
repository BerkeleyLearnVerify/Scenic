
import math

import lgsvl
import dreamview
import numpy as np
from scipy import linalg

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
        self.usingApollo = False

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

            # Initialize Apollo if needed
            if hasattr(obj, 'apolloVehicle'):
                self.initApolloFor(obj, lgsvlObj)

        # TODO reset object controllers???

    def groundElevationAt(self, pos):
        origin = utils.scenicToLGSVLPosition(pos, 100000)
        result = self.client.raycast(origin, lgsvl.Vector(0, -1, 0), 1)
        if result is None:
            print(f'WARNING: no ground at position {pos}')
            return 0
        return result.point.y

    def initApolloFor(self, obj, lgsvlObj):
        """Initialize Apollo for an ego vehicle.

        Uses LG's interface which injects packets into Dreamview."""
        if self.usingApollo:
            raise RuntimeError('can only use one Apollo vehicle')
        self.usingApollo = True

        # connect bridge from LGSVL to Apollo
        lgsvlObj.connect_bridge(obj.bridgeHost, obj.bridgePort)

        # set up connection and map/vehicle configuration
        dv = dreamview.Connection(self.client, lgsvlObj)
        obj.dreamview = dv
        waitToStabilize = False
        hdMap = self.scene.params['apolloHDMap']
        if dv.getCurrentMap() != hdMap:
            dv.setHDMap(hdMap)
            waitToStabilize = True
        if dv.getCurrentVehicle() != obj.apolloVehicle:
            dv.setVehicle(obj.apolloVehicle)
            waitToStabilize = True
        
        print('Initializing Apollo...')

        # stop the car to cancel buffered speed from previous simulations
        cntrl = lgsvl.VehicleControl()
        cntrl.throttle = 0.0
        lgsvlObj.apply_control(cntrl, True)
        # start modules
        dv.disableModule('Control')
        for module in obj.apolloModules:
            dv.enableModule(module)
        while True:
            ready = dv.getModuleStatus()
            if all(ready[module] for module in obj.apolloModules):
                break

        # wait for Apollo to stabilize, if needed
        if waitToStabilize:
            self.client.run(15)
        dv.enableModule('Control')
        print('Initialized Apollo.')

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
            heading = utils.lgsvlToScenicRotation(state.rotation, tolerance2D=15)
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
            agentType = obj.lgsvlAgentType
            if agentType in (lgsvl.AgentType.NPC, lgsvl.AgentType.PEDESTRIAN):
                lgsvlObject.follow(self.waypoints)
            else:
                raise RuntimeError('used FollowWaypointsAction with'
                                   f' unsupported agent {lgsvlObject}')
        self.lastTime = sim.currentTime

class CancelWaypointsAction(simulators.Action):
    def applyTo(self, obj, lgsvlObject, sim):
        lgsvlObject.walk_randomly(False)

class SetDestinationAction(simulators.Action):
    def __init__(self, dest):
        self.dest = dest
        self.timer = 0


    def applyTo(self, obj, lgsvlObject, sim):
        if self.timer == 0:
            print('Setting destination...')
            z = sim.groundElevationAt(self.dest)
            obj.dreamview.setDestination(self.dest.x, self.dest.y, z,
                                      coordType=dreamview.CoordType.Unity)
        # push vehicle for 1 second to start
        oneSec = int(1.0/sim.timeStep)
        if self.timer < oneSec:
            cntrl = lgsvl.VehicleControl()
            cntrl.throttle = 0.5
            lgsvlObject.apply_control(cntrl, True)
        elif self.timer == oneSec:
            print('Autopilot...')
            cntrl = lgsvl.VehicleControl()
            cntrl.throttle = 0.5
            lgsvlObject.apply_control(cntrl, False)
        self.timer = self.timer + 1


        

