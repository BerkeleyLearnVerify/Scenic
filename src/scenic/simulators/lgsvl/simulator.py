
import lgsvl

import scenic.core.simulators as simulators
import scenic.simulators.lgsvl.utils as utils
import scenic.syntax.veneer as veneer
from scenic.core.vectors import Vector
import math

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
        self.data = {}
        self.collisionOccurred = False

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
            
            # Initialize Data
            self.data[obj] = {}
            # Initialize Apollo if needed
            if getattr(obj, 'apolloVehicle', None):
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

        def on_collision(agent1, agent2, contact):
            if agent1 is not None and agent1.name == lgsvlObj.name:
                self.data[obj]['collision'] = True
            if agent2 is not None and agent2.name == lgsvlObj.name:
                self.data[obj]['collision'] = True
            if self.data[obj]['collision']:
                self.collisionOccurred = True
                print('COLLISION !')

        # Initialize Data
        self.data[obj]['collision'] = False
        lgsvlObj.on_collision(on_collision)

        # connect bridge from LGSVL to Apollo
        lgsvlObj.connect_bridge(obj.bridgeHost, obj.bridgePort)

        # set up connection and map/vehicle configuration
        import dreamview
        dv = dreamview.Connection(self.client, lgsvlObj)
        obj.dreamview = dv
        waitToStabilize = False
        hdMap = self.scene.params['apolloHDMap']
        if dv.getCurrentMap() != hdMap:
            print('previous map:',dv.getCurrentMap())
            print('current map:',hdMap)
            dv.setHDMap(hdMap)
            waitToStabilize = True
            print('map updated')
        if dv.getCurrentVehicle() != obj.apolloVehicle:
            dv.setVehicle(obj.apolloVehicle)
            waitToStabilize = True
            print('vehicle updated')
        
        print('Initializing Apollo...')

        # stop the car to cancel buffered speed from previous simulations
        cntrl = lgsvl.VehicleControl()
        cntrl.throttle = 0.0
        lgsvlObj.apply_control(cntrl, True)
        # start modules
        dv.disableModule('Control')
        #for module in obj.apolloModules:
        #    dv.disableModule(module)
        #self.client.run(5)
        ready = dv.getModuleStatus()
        for module in obj.apolloModules:
            if not ready[module]:
                dv.enableModule(module)
                print('Module', module, 'is not ready...')
                waitToStabilize = True
        while True:
            ready = dv.getModuleStatus()
            if all(ready[module] for module in obj.apolloModules):
                break

        # wait for Apollo to stabilize, if needed
        if waitToStabilize:
            print('waiting to stabilize')
            self.client.run(25)
        dv.enableModule('Control')
        self.client.run(15)
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
            # speed
            vel = utils.lgsvlToScenicPosition(state.velocity)
            obj.speed = math.hypot(*vel)

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

