"""Interface to Webots for dynamic simulations.

This interface is intended to be instantiated from inside the controller script
of a Webots `Robot node`_ with the ``supervisor`` field set to true. Such a
script can create a `WebotsSimulator` (passing in a reference to the supervisor
node) and then call its `simulate` method as usual to run a simulation. For an
example, see :file:`examples/webots/generic/controllers/scenic_supervisor.py`.

Scenarios written for this interface should use our generic Webots world model
:doc:`scenic.simulators.webots.model` or a model derived from it. Objects which
are instances of `WebotsObject` will be matched to Webots nodes; see the model
documentation for details.

.. _Robot node: https://www.cyberbotics.com/doc/reference/robot
"""

from collections import defaultdict
import math

from scenic.core.simulators import Simulator, Simulation, Action
from scenic.core.vectors import Vector
from scenic.simulators.webots.utils import WebotsCoordinateSystem, ENU

class WebotsSimulator(Simulator):
    """`Simulator` object for Webots.

    Args:
        supervisor: Supervisor node handle from the Webots Python API.
    """
    def __init__(self, supervisor):
        super().__init__()
        self.supervisor = supervisor
        topLevelNodes = supervisor.getRoot().getField('children')
        worldInfo = None
        for i in range(topLevelNodes.getCount()):
            child = topLevelNodes.getMFNode(i)
            if child.getTypeName() == 'WorldInfo':
                worldInfo = child
                break
        if not worldInfo:
            raise RuntimeError('Webots world does not contain a WorldInfo node')
        system = worldInfo.getField('coordinateSystem').getSFString()
        self.coordinateSystem = WebotsCoordinateSystem(system)

    def createSimulation(self, scene, verbosity=0):
        return WebotsSimulation(scene, self.supervisor,
                                coordinateSystem=self.coordinateSystem)

class WebotsSimulation(Simulation):
    """`Simulation` object for Webots.

    Attributes:
        supervisor: Webots supervisor node used for the simulation. This is
            exposed for the use of scenarios which need to call Webots APIs
            directly; e.g. :samp:`simulation().supervisor.setLabel({...})`.
    """
    def __init__(self, scene, supervisor, verbosity=0, coordinateSystem=ENU):
        timestep = supervisor.getBasicTimeStep() / 1000
        super().__init__(scene, timestep=timestep, verbosity=verbosity)
        self.supervisor = supervisor
        self.coordinateSystem = coordinateSystem
        self.objects = scene.objects

        # Find Webots objects corresponding to Scenic objects
        self.webotsObjects = {}
        usedNames = defaultdict(lambda: 0)
        for obj in self.objects:
            if not hasattr(obj, 'webotsName'):
                continue    # not a Webots object
            if obj.webotsName:
                name = obj.webotsName
            else:
                ty = obj.webotsType
                if not ty:
                    raise RuntimeError(f'object {obj} has no webotsName or webotsType')
                nextID = usedNames[ty]
                usedNames[ty] += 1
                if nextID == 0 and supervisor.getFromDef(ty):
                    name = ty
                else:
                    name = f'{ty}_{nextID}'
            webotsObj = supervisor.getFromDef(name)
            if webotsObj is None:
                raise RuntimeError(f'Webots object {name} does not exist in world')
            self.webotsObjects[obj] = webotsObj
            obj.webotsObject = webotsObj
            obj.webotsName = name

            # get starting elevation
            if obj.elevation is None:
                pos = webotsObj.getField('translation').getSFVec3f()
                spos = self.coordinateSystem.positionToScenic(pos)
                obj.elevation = spos[2]

        # Reset Webots simulation
        supervisor.simulationResetPhysics()

        # Set initial properties of Webots objects
        self.writePropertiesToWebots()

    def writePropertiesToWebots(self):
        for obj, webotsObj in self.webotsObjects.items():
            # position
            pos = self.coordinateSystem.positionFromScenic(
                obj.position + obj.positionOffset,
                elevation=obj.elevation
            )
            webotsObj.getField('translation').setSFVec3f(pos)
            # heading
            rot = self.coordinateSystem.rotationFromScenic(
                obj.heading + obj.rotationOffset
            )
            webotsObj.getField('rotation').setSFRotation(rot)
            # battery
            battery = getattr(obj, 'battery', None)
            if battery:
                if not isinstance(battery, (tuple, list)) or len(battery) != 3:
                    raise RuntimeError(f'"battery" of {obj.webotsName} does not'
                                       ' have 3 components')
                field = webotsObj.getField('battery')
                field.setMFFloat(0, battery[0])
                field.setMFFloat(1, battery[1])
                field.setMFFloat(2, battery[2])
            # customData
            customData = getattr(obj, 'customData', None)
            if customData:
                if not isinstance(customData, str):
                    raise RuntimeError(f'"customData" of {obj.webotsName} is not a string')
                webotsObj.getField('customData').setSFString(customData)
            # controller
            if obj.controller:
                controllerField = webotsObj.getField('controller')
                curCont = controllerField.getSFString()
                if obj.controller != curCont:
                    # the following operation also causes the controller to be restarted
                    controllerField.setSFString(obj.controller)
                elif obj.resetController:
                    webotsObj.restartController()

    def createObjectInSimulator(self, obj):
        raise RuntimeError('the Webots interface does not support dynamic object creation')

    def step(self):
        ms = round(1000 * self.timestep)
        self.supervisor.step(ms)

    def getProperties(self, obj, properties):
        webotsObj = self.webotsObjects.get(obj)
        if not webotsObj:   # static object with no Webots counterpart
            return { prop: getattr(obj, prop) for prop in properties }

        pos = webotsObj.getField('translation').getSFVec3f()
        x, y, elevation = self.coordinateSystem.positionToScenic(pos)
        rot = webotsObj.getField('rotation').getSFRotation()
        heading = self.coordinateSystem.rotationToScenic(rot)
        lx, ly, lz, ax, ay, az = webotsObj.getVelocity()
        vx, vy, vz = self.coordinateSystem.positionToScenic((lx, ly, lz))
        velocity = (vx, vy)
        speed = math.hypot(*velocity)

        values = dict(
            position=Vector(x, y),
            elevation=elevation,
            heading=heading,
            velocity=velocity,
            speed=speed,
            angularSpeed=ay,
        )

        if hasattr(obj, 'battery'):
            field = webotsObj.getField('battery')
            val = (field.getMFFloat(0), obj.battery[1], obj.battery[2])
            values['battery'] = val

        return values
