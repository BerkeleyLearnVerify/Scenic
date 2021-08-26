"""Actions for dynamic agents in Webots simulations."""

import pickle

from scenic.core.simulators import Action
import scenic.simulators.webots.utils as utils

class OffsetAction(Action):
    """Move an object by the given offset relative to its current heading."""
    def __init__(self, offset):
        self.offset = offset

    def applyTo(self, obj, webotsObj):
        pos = obj.position.offsetRotated(obj.heading, self.offset)
        pos = utils.scenicToWebotsPosition(pos, y=obj.elevation)
        webotsObj.getField('translation').setSFVec3f(pos)

class WriteFileAction(Action):
    """Pickle the given data and write the result to a file.

    For use in communication with external controllers or other code.
    """
    def __init__(self, path, data):
        self.path = path
        self.data = data

    def applyTo(self, obj, webotsObj):
        with open(self.path, 'wb') as outFile:
            pickle.dump(self.data, outFile)
