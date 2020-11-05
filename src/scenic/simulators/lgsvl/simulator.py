
import math
import warnings

import lgsvl

import scenic.core.simulators as simulators
import scenic.simulators.lgsvl.utils as utils
from scenic.syntax.veneer import verbosePrint
from scenic.core.vectors import Vector

class LGSVLSimulator(simulators.Simulator):
    def __init__(self, lgsvl_scene, address='localhost', port=8181, alwaysReload=False):
        super().__init__()
        verbosePrint('Connecting to LGSVL Simulator...')
        self.client = lgsvl.Simulator(address=address, port=port)
        if alwaysReload or self.client.current_scene != lgsvl_scene:
            self.client.load(scene=lgsvl_scene)
        verbosePrint('Map loaded in simulator.')

    def createSimulation(self, scene, verbosity=0):
        return LGSVLSimulation(scene, self.client, verbosity=verbosity)


class LGSVLSimulation(simulators.Simulation):
    def __init__(self, scene, client, verbosity=0):
        timestep = scene.params.get('time_step', 1.0/10)
        super().__init__(scene, timestep=timestep, verbosity=verbosity)
        self.client = client
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
            elevation = obj.elevation
            if elevation is None:
                elevation = self.groundElevationAt(obj.position)
            state.transform.position = utils.scenicToLGSVLPosition(obj.position, elevation)
            state.transform.rotation = utils.scenicToLGSVLRotation(obj.heading)

            # Create LGSVL object
            lgsvlObj = self.client.add_agent(name, agentType, state)
            obj.lgsvlObject = lgsvlObj
            
            # Initialize Data
            self.data[obj] = {}
            # Initialize Apollo if needed
            if getattr(obj, 'apolloVehicle', None):
                self.initApolloFor(obj, lgsvlObj)

    def groundElevationAt(self, pos):
        origin = utils.scenicToLGSVLPosition(pos, 100000)
        result = self.client.raycast(origin, lgsvl.Vector(0, -1, 0), 1)
        if result is None:
            warnings.warn(f'no ground at position {pos}')
            return 0
        return result.point.y

    def initApolloFor(self, obj, lgsvlObj):
        """Initialize Apollo for an ego vehicle.

        Uses LG's interface which injects packets into Dreamview.
        """
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

        # Initialize Data
        self.data[obj]['collision'] = False
        lgsvlObj.on_collision(on_collision)

        # connect bridge from LGSVL to Apollo
        lgsvlObj.connect_bridge(obj.bridgeHost, obj.bridgePort)

        # set up connection and map/vehicle configuration
        import lgsvl.dreamview as dreamview
        dv = dreamview.Connection(self.client, lgsvlObj)
        obj.dreamview = dv
        waitToStabilize = False
        hdMap = self.scene.params['apolloHDMap']
        if dv.get_current_map() != hdMap:
            dv.set_hd_map(hdMap)
            waitToStabilize = True
        if dv.get_current_vehicle() != obj.apolloVehicle:
            dv.set_vehicle(obj.apolloVehicle)
            waitToStabilize = True
        
        verbosePrint('Initializing Apollo...')

        # stop the car to cancel buffered speed from previous simulations
        cntrl = lgsvl.VehicleControl()
        cntrl.throttle = 0.0
        lgsvlObj.apply_control(cntrl, True)
        # start modules
        dv.disable_module('Control')
        ready = dv.get_module_status()
        for module in obj.apolloModules:
            if not ready[module]:
                dv.enable_module(module)
                verbosePrint(f'Module {module} is not ready...')
                waitToStabilize = True
        while True:
            ready = dv.get_module_status()
            if all(ready[module] for module in obj.apolloModules):
                break

        # wait for Apollo to stabilize, if needed
        if waitToStabilize:
            verbosePrint('Waiting for Apollo to stabilize...')
            self.client.run(25)
        dv.enable_module('Control')
        self.client.run(15)
        verbosePrint('Initialized Apollo.')

    def executeActions(self, allActions):
        super().executeActions(allActions)

        # Apply state/control updates which were accumulated while executing the actions
        for obj in self.agents:
            if obj._stateUpdated:
                obj.lgsvlObject.state = obj.state
                obj._stateUpdated = False
            ctrl = getattr(obj, '_control', None)
            if ctrl is not None:
                obj.lgsvlObject.apply_control(ctrl, obj._stickyControl)
                obj._control = None

    def step(self):
        self.client.run(time_limit=self.timestep)

    def getProperties(self, obj, properties):
        lgsvlObj = obj.lgsvlObject
        state = lgsvlObj.state
        obj.state = state   # cache state for subsequent updates

        velocity = utils.lgsvlToScenicPosition(state.velocity)
        speed = math.hypot(*velocity)

        values = dict(
            position=utils.lgsvlToScenicPosition(state.position),
            elevation=utils.lgsvlToScenicElevation(state.position),
            heading=utils.lgsvlToScenicRotation(state.rotation),
            velocity=velocity,
            speed=speed,
            angularSpeed=utils.lgsvlToScenicAngularSpeed(state.rotation),
        )
        return values
