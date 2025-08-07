from scenic.core.sensors import Sensor


class MetaDriveRGBSensor(Sensor):
    def getObservation(self):
        # could pass in sensor attrs here in perceive() or in track() in createobjinsim
        img = self.metadrive_sensor.perceive()
        print(img)
        return img


class MetaDriveSSSensor(Sensor):
    def getObservation(self):
        return "rgb image"


# class MetaDriveRGBSensor(Sensor):
#     def __init__(
#             self,
#             offset=(1.6, 0.0, 1.7),
#             hpr=(0.0, 0.0, 0.0),
#             to_float=False
#         ):
#             super().__init__()
#             self.actor = None          # will be set in createObjectInSimulator
#             self.client = None         # will be set in createObjectInSimulator
#             self.offset = offset       # camera offset from actor
#             self.hpr = hpr             # heading, pitch, roll relative to actor
#             self.to_float = to_float   # output format of the image

#     def getObservation(self):
#         if self.actor is None or self.client is None:
#             raise RuntimeError("MetaDriveRGBSensor is not fully initialized")

#         cam = self.client.engine.get_sensor("rgb")
#         return cam.perceive(
#             new_parent_node=self.actor.origin,
#             position=self.offset,
#             hpr=self.hpr,
#             to_float=self.to_float
#         )

# class MetaDriveSSSensor(Sensor):
#     def __init__(
#             self,
#             offset=(1.6, 0.0, 1.7),
#             hpr=(0.0, 0.0, 0.0),
#             to_float=False
#         ):
#             super().__init__()
#             self.actor = None          # will be set in createObjectInSimulator
#             self.client = None         # will be set in createObjectInSimulator
#             self.offset = offset       # camera offset from actor
#             self.hpr = hpr             # heading, pitch, roll relative to actor
#             self.to_float = to_float   # output format of the image

#     def getObservation(self):
#         if self.actor is None or self.client is None:
#             raise RuntimeError("MetaDriveRGBSensor is not fully initialized")

#         cam = self.client.engine.get_sensor("rgb")
#         return cam.perceive(
#             new_parent_node=self.actor.origin,
#             position=self.offset,
#             hpr=self.hpr,
#             to_float=self.to_float
#         )
