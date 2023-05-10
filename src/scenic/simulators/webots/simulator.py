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
import tempfile
from os import path
from textwrap import dedent
import ctypes

import trimesh
from scenic.core.regions import MeshVolumeRegion
from scenic.core.object_types import Object2D
from scenic.core.simulators import Simulator, Simulation
from scenic.core.vectors import Vector
from scenic.simulators.webots.utils import WebotsCoordinateSystem, ENU
from scenic.core.type_support import toOrientation

class WebotsSimulator(Simulator):
    """`Simulator` object for Webots.

    Args:
        supervisor: Supervisor node handle from the Webots Python API.
    """
    def __init__(self, supervisor, timestep=None):
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
        self.timestep = timestep

    def createSimulation(self, scene, verbosity=0):
        return WebotsSimulation(scene, self.supervisor,
                                coordinateSystem=self.coordinateSystem,
                                timestep=self.timestep)

class WebotsSimulation(Simulation):
    """`Simulation` object for Webots.

    Attributes:
        supervisor: Webots supervisor node used for the simulation. This is
            exposed for the use of scenarios which need to call Webots APIs
            directly; e.g. :scenic:`simulation().supervisor.setLabel({...})`.
    """
    def __init__(self, scene, supervisor, verbosity=0, coordinateSystem=ENU, timestep=None):
        timestep = supervisor.getBasicTimeStep() / 1000 if timestep is None else timestep
        super().__init__(scene, timestep=timestep, verbosity=verbosity)
        self.supervisor = supervisor
        self.coordinateSystem = coordinateSystem
        self.objects = scene.objects

        # Find Webots objects corresponding to Scenic objects
        self.webotsObjects = {}
        usedNames = defaultdict(lambda: 0)

        # directory to store proto files for adhoc webots objects
        tmpMeshDir = tempfile.mkdtemp()
        # set to store
        adhocObjectId = 1

        for obj in self.objects:
            # make sure `obj` is a Webots object
            if not hasattr(obj, 'webotsName'):
                continue    # not a Webots object
            
            # get a name of the corresponding webots object
            name = None
            if obj.webotsAdhoc:
                # dynamically generate object from Scenic object mesh
                if not hasattr(obj.shape, "mesh"):
                    raise RuntimeError(f'Cannot dynamically instantiate shape without mesh')
                
                objFilePath = path.join(tmpMeshDir, f"{adhocObjectId}.obj")

                objectRawMesh = obj.shape.mesh
                objectScaledMesh = MeshVolumeRegion(mesh=objectRawMesh, dimensions=(obj.width, obj.length, obj.height)).mesh

                trimesh.exchange.export.export_mesh(objectScaledMesh, objFilePath)

                name = f"SCENIC_ADHOC_{adhocObjectId}"
                
                rootNode = supervisor.getRoot()
                rootChildrenField = rootNode.getField("children")

                protoDef = ""
                if isPhysicsEnabled(obj):
                    protoDef = dedent(f"""
                        DEF {name} ScenicObjectWithPhysics {{
                            url "{objFilePath}"
                        }}
                        """
                    )
                else:
                    protoDef = dedent(f"""
                        DEF {name} ScenicObject {{
                            url "{objFilePath}"
                        }}
                        """
                    )

                rootChildrenField.importMFNodeFromString(-1, protoDef)
                adhocObjectId += 1
            else:
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

            # 2D Compatibility - Import starting elevation
            if isinstance(obj, Object2D) and obj.elevation is None:
                pos = webotsObj.getField('translation').getSFVec3f()
                spos = self.coordinateSystem.positionToScenic(pos)
                obj.elevation = spos[2]

        # subtract 1 to get the number of adhoc objects in the world
        self.adhocObjectCount = adhocObjectId - 1

        # Reset Webots simulation
        supervisor.simulationResetPhysics()

        # Set initial properties of Webots objects
        self.writePropertiesToWebots()

    def writePropertiesToWebots(self):
        for obj, webotsObj in self.webotsObjects.items():
            # position
            if isinstance(obj, Object2D):
                # 2D Compatibility - Overwrite z value with elevation
                pos = self.coordinateSystem.positionFromScenic(
                    Vector(obj.position.x, obj.position.y, obj.elevation) + obj.positionOffset
                )
                webotsObj.getField('translation').setSFVec3f(pos)
            else:
                pos = self.coordinateSystem.positionFromScenic(
                    obj.position + obj.positionOffset
                )
                webotsObj.getField('translation').setSFVec3f(pos)

            # orientation
            offsetOrientation = toOrientation(obj.rotatioOffset)
            webotsObj.getField("rotation").setSFRotation(
                self.coordinateSystem.orientationFromScenic(obj.orientation, offsetOrientation)
            )

            # density
            densityField = getFieldSafe(webotsObj, "density")
            if densityField is not None and hasattr(obj, "density") and obj.density is not None:
                densityField.setSFFloat(float(obj.density))

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
        x, y, z = self.coordinateSystem.positionToScenic(pos)
        lx, ly, lz, ax, ay, az = webotsObj.getVelocity()
        vx, vy, vz = self.coordinateSystem.positionToScenic((lx, ly, lz))
        velocity = Vector(vx, vy, vz)
        speed = math.hypot(*velocity)
        angularSpeed = math.hypot(ax, ay, az)

        offsetOrientation = toOrientation(obj.rotationOffset)
        orientation = self.coordinateSystem.orientationToScenic(
            webotsObj.getField('rotation').getSFRotation(),
            offsetOrientation,
        ) * obj.parentOrientation.inverse

        densityField = getFieldSafe(webotsObj, "density")
        density = None
        if densityField:
            density = densityField.getSFFloat()

        values = dict(
            position=Vector(x, y, z),
            velocity=velocity,
            speed=speed,
            angularSpeed=angularSpeed,
            angularVelocity=Vector(ax, ay, az),
            yaw=orientation.yaw,
            pitch=orientation.pitch,
            roll=orientation.roll,
            density=density,
            elevation=z
        )

        if hasattr(obj, 'battery'):
            field = webotsObj.getField('battery')
            val = (field.getMFFloat(0), obj.battery[1], obj.battery[2])
            values['battery'] = val

        return values

    def destroy(self):
        """Destroy adhoc objects generated at the beginning of the simulation"""
        for i in range(1, self.adhocObjectCount + 1):
            name = self._getAdhocObjectName(i)
            node = self.supervisor.getFromDef(name)
            node.remove()

    def _getAdhocObjectName(self, i: int) -> str:
        return f"SCENIC_ADHOC_{i}"

def f(x: int) -> int:
    """_summary_

    Args:
        x (int): _description_

    Returns:
        int: _description_
    """
    return x + x

def getFieldSafe(webotsObject, fieldName):
    """Get field from webots object. Return null if no such field exists.
    Needed to workaround this issue (https://github.com/cyberbotics/webots/issues/5646)

    Args:
        webotsObject: webots object
        fieldName: name of the field to look for

    Returns:
        Field|None: Field object if the field with the given name exists. None otherwise.
    """

    field = webotsObject.getField(fieldName)
    # this seems to always return some object, but return None if field is None
    if field is None:
        return None
    
    # if field is valid, it has a valid pointer
    if isinstance(field._ref, ctypes.c_void_p) and field._ref.value is not None:
        # then the field is valid and we return the reference
        return field
    
    # if the pointer points to None, then the field does not exist on this object
    return None


def isPhysicsEnabled(webotsObject):
    if isinstance(webotsObject.webotsAdhoc, bool):
        return webotsObject
    if isinstance(webotsObject.webotsAdhoc, dict):
        return webotsObject.webotsAdhoc.get("physics", True)
    raise RuntimeError(f"webotsAdhoc must be a boolean or dictionary")
