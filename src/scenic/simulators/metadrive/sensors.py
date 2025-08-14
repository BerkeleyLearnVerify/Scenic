from scenic.core.sensors import Sensor


class MetaDriveVisionSensor(Sensor):
    """
    MetaDrive camera mount parameters.

    position: (x, y, z) in meters, relative to the parent object's origin (parent node).
      - +X = right, +Y = forward, +Z = up  (Panda3D / MetaDrive, Z-up, right-handed)

    hpr: (heading, pitch, roll) in degrees.
      - H (yaw) about Z: + turns left, - turns right
      - P (pitch) about X: + tilts up, - tilts down
      - R (roll) about Y: + roll left, - rolls right

    Defaults match MetaDrive:
      position=(0.0, 0.8, 1.5), hpr=(0.0, -5.0, 0.0), size=84×84
    """

    DEFAULT_POS = (0.0, 0.8, 1.5)
    DEFAULT_HPR = (0.0, -5.0, 0.0)
    DEFAULT_SIZE = (84, 84)

    def __init__(self, offset=None, rotation=None, width=None, height=None):
        self.offset = self.DEFAULT_POS if offset is None else offset
        self.rotation = self.DEFAULT_HPR if rotation is None else rotation
        self.width = self.DEFAULT_SIZE[0] if width is None else width
        self.height = self.DEFAULT_SIZE[1] if height is None else height
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
