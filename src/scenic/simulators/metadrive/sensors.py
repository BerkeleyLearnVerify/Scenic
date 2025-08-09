from scenic.core.sensors import Sensor


class MetaDriveRGBSensor(Sensor):
    def __init__(self):
        self.metadrive_sensor = None  # will be set by the simulator

    def getObservation(self):
        if self.metadrive_sensor is None:
            raise RuntimeError(
                "MetaDriveRGBSensor not attached; simulator hasn't set metadrive_sensor yet."
            )

        # two options for getting image: 1: get_image but need to pass in obj each time but returns correct image format OR
        # 2: perceive(), dont need to pass in obj again, but will need to change format

        # returns NumPy uint8 in [0..255] (shape (H, W, 3), BGR).
        img_bgr = self.metadrive_sensor.perceive(to_float=False)
        # BGR → RGB
        img_rgb = img_bgr[..., ::-1]
        print("IMAGE!!!!", img_rgb)
        return img_rgb


class MetaDriveSSSensor(Sensor):
    def __init__(self):
        self.metadrive_sensor = None  # will be set by the simulator

    def getObservation(self):
        if self.metadrive_sensor is None:
            raise RuntimeError(
                "MetaDriveRGBSensor not attached; simulator hasn't set metadrive_sensor yet."
            )

        # returns NumPy uint8 in [0..255] (shape (H, W, 3), BGR).
        img_bgr = self.metadrive_sensor.perceive(to_float=False)
        # BGR → RGB
        img_rgb = img_bgr[..., ::-1]
        print("IMAGE!!!!", img_rgb)
        return img_rgb
