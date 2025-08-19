from scenic.core.sensors import RGBSensor, Sensor, SSSensor

# NOTE: MetaDrive/Panda3D coords:
#   position (x, y, z) m relative to the parent object's orgin: +X right, +Y forward, +Z up (Z-up, RH).
#   rotation (Heading, Pitch, Roll) degrees: H=yaw about Z (+left), P=pitch about X (+up), R=roll about Y (+left).


class MetaDriveVisionSensor(Sensor):
    def __init__(
        self,
        offset=None,
        rotation=(0, 0, 0),
        width=None,
        height=None,
        attributes=None,
    ):
        if width is None or height is None:
            raise ValueError("width and height are required for sensors")
        self.offset = offset
        self.rotation = rotation
        self.width = width
        self.height = height
        self.attributes = attributes or {}
        self.metadrive_sensor = None  # set by the simulator

    def getObservation(self):
        if self.metadrive_sensor is None:
            raise RuntimeError(
                "MetaDrive sensor not attached; (metadrive_sensor not set by simulator)."
            )
        # MetaDrive returns BGR; Scenic expects RGB.
        img_bgr = self.metadrive_sensor.perceive(to_float=False)
        return img_bgr[..., ::-1]


class MetaDriveRGBSensor(MetaDriveVisionSensor):
    pass


class MetaDriveSSSensor(MetaDriveVisionSensor):
    pass
