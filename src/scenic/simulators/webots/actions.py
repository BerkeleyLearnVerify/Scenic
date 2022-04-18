"""Actions for dynamic agents in Webots simulations."""

import pickle

from scenic.core.simulators import Action
from scenic.core.type_support import toScalar, toVector
import scenic.simulators.webots.utils as utils

class OffsetAction(Action):
    """Move an object by the given offset relative to its current heading."""
    def __init__(self, offset):
        self.offset = toVector(offset, 'OffsetAction with non-vector offset')

    def applyTo(self, obj, sim):
        pos = obj.position.offsetRotated(obj.heading, self.offset)
        pos = sim.coordinateSystem.positionFromScenic(pos, elevation=obj.elevation)
        obj.webotsObject.getField('translation').setSFVec3f(pos)

class ApplyForceAction(Action):
    """Apply a given force to the object."""
    def __init__(self, force, relative=False):
        self.force = toVector(force, 'ApplyForceAction with non-vector force')
        self.relative = relative

    def applyTo(self, obj, sim):
        force = sim.coordinateSystem.positionFromScenic(self.force)
        obj.webotsObject.addForce(force, self.relative)

class WriteFileAction(Action):
    """Pickle the given data and write the result to a file.

    For use in communication with external controllers or other code.
    """
    def __init__(self, path, data):
        self.path = path
        self.data = data

    def applyTo(self, obj, sim):
        with open(self.path, 'wb') as outFile:
            pickle.dump(self.data, outFile)
