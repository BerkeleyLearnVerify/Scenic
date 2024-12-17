from scenic.core.simulators import Action
from scenic.core.type_support import toVector

from .utils import (
    VectorToAirsimVec,
    airsimToScenicLocation,
    airsimToScenicOrientation,
    scenicToAirsimLocation,
    scenicToAirsimOrientation,
    scenicToAirsimScale,
)


class SetVelocity(Action):
    def __init__(self, velocity):
        print("vel=", velocity)
        self.newVelocity = airsimToScenicLocation(VectorToAirsimVec(velocity))

    def applyTo(self, obj, sim):
        client = sim.client
        client.cancelLastTask(vehicle_name=obj.realObjName)
        client.moveByVelocityAsync(
            self.newVelocity.x,
            self.newVelocity.y,
            self.newVelocity.z,
            duration=5,
            vehicle_name=obj.realObjName,
        )
