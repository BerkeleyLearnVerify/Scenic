"""Dynamic simulator interface for LGSVL."""

import math
import time
import warnings

import lgsvl

import scenic.core.simulators as simulators
from scenic.core.vectors import Vector
import scenic.simulators.lgsvl.utils as utils
from scenic.syntax.veneer import verbosePrint


class LGSVLSimulator(simulators.Simulator):
    """A connection to an instance of LGSVL.

    See the `SVL documentation`_ for details on how to set the parameters below.

    Uses a default timestep of 0.1 seconds.

    Args:
        sceneID (str): Identifier for the map ("scene") to load in SVL.
        address (str): Address where SVL is running.
        port (int): Port on which to connect to SVL.
        alwaysReload (bool): Whether to force reloading the map upon connecting,
            even if the simulator already has the desired map loaded.

    .. _SVL documentation: https://www.svlsimulator.com/docs/python-api/python-api
    """

    def __init__(self, sceneID, address="localhost", port=8181, alwaysReload=False):
        super().__init__()
        verbosePrint("Connecting to LGSVL Simulator...")
        self.client = lgsvl.Simulator(address=address, port=port)
        if alwaysReload or self.client.current_scene != sceneID:
            self.client.load(scene=sceneID)
        verbosePrint("Map loaded in simulator.")

    def createSimulation(self, scene, **kwargs):
        return LGSVLSimulation(scene, self.client, **kwargs)


class LGSVLSimulation(simulators.Simulation):
    """Subclass of `Simulation` for LGSVL."""

    def __init__(self, scene, client, *, timestep, **kwargs):
        self.client = client
        self.usingApollo = False
        self.data = {}
        self.collisionOccurred = False

        if timestep is None:
            timestep = scene.params.get("time_step", 0.1)  # backwards-compatibility

        super().__init__(scene, timestep=timestep, **kwargs)

    def setup(self):
        # Reset simulator (deletes all existing objects)
        self.client.reset()

        super().setup()

    def createObjectInSimulator(self, obj):
        if not hasattr(obj, "lgsvlObject"):
            return  # not an LGSVL object

        # Figure out what type of LGSVL object this is
        if not hasattr(obj, "lgsvlName"):
            raise RuntimeError(f"object {obj} does not have an lgsvlName property")
        if not hasattr(obj, "lgsvlAgentType"):
            raise RuntimeError(f"object {obj} does not have an lgsvlAgentType property")
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
        if getattr(obj, "apolloVehicle", None):
            self.initApolloFor(obj, lgsvlObj)

    def groundElevationAt(self, pos):
        origin = utils.scenicToLGSVLPosition(pos, 100000)
        result = self.client.raycast(origin, lgsvl.Vector(0, -1, 0), 1)
        if result is None:
            warnings.warn(
                f"no ground at position {pos}", simulators.SimulatorInterfaceWarning
            )
            return 0
        return result.point.y

    def initApolloFor(self, obj, lgsvlObj):
        """Initialize Apollo for an ego vehicle.

        Uses LG's interface which injects packets into Dreamview.
        """
        if self.usingApollo:
            raise RuntimeError("can only use one Apollo vehicle")
        self.usingApollo = True

        def on_collision(agent1, agent2, contact):
            if agent1 is not None and agent1.name == lgsvlObj.name:
                self.data[obj]["collision"] = True
            if agent2 is not None and agent2.name == lgsvlObj.name:
                self.data[obj]["collision"] = True
            if self.data[obj]["collision"]:
                self.collisionOccurred = True

        # Initialize Data
        self.data[obj]["collision"] = False
        lgsvlObj.on_collision(on_collision)

        # connect bridge from LGSVL to Apollo
        lgsvlObj.connect_bridge(obj.bridgeHost, obj.bridgePort)

        # set up connection and map/vehicle configuration
        from lgsvl import dreamview

        dv = dreamview.Connection(self.client, lgsvlObj)
        obj.dreamview = dv
        hdMap = self.scene.params["apolloHDMap"]
        dv.set_hd_map(hdMap)
        dv.set_vehicle(obj.apolloVehicle)

    def executeActions(self, allActions):
        super().executeActions(allActions)

        # Apply state/control updates which were accumulated while executing the actions
        for obj in self.agents:
            if obj._stateUpdated:
                obj.lgsvlObject.state = obj.state
                obj._stateUpdated = False
            ctrl = getattr(obj, "_control", None)
            if ctrl is not None:
                obj.lgsvlObject.apply_control(ctrl, obj._stickyControl)
                obj._control = None

    def step(self):
        self.client.run(time_limit=self.timestep)

    def getProperties(self, obj, properties):
        lgsvlObj = obj.lgsvlObject
        state = lgsvlObj.state
        obj.state = state  # cache state for subsequent updates

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

    def destroy(self):
        self.client.reset()
