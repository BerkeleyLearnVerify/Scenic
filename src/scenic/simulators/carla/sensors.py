import numpy as np

from scenic.core.sensors import CallbackSensor, RGBSensor, SSSensor


class CarlaVisionSensor(CallbackSensor):
    def __init__(
        self,
        offset=(0, 0, 0),
        rotation=(0, 0, 0),
        width=None,
        height=None,
        attributes=None,
    ):
        super().__init__()
        self.offset = offset
        self.rotation = rotation

        if isinstance(attributes, str):
            raise NotImplementedError(
                "String parsing for attributes is not yet implemented. Feel free to do so."
            )
        elif isinstance(attributes, dict):
            self.attributes = attributes
        else:
            self.attributes = {}

        if width is not None:
            self.attributes["image_size_x"] = int(width)
        if height is not None:
            self.attributes["image_size_y"] = int(height)

        self.convert = None
        convert = self.attributes.get("convert")
        if convert is not None and not isinstance(convert, (str, int)):
            raise TypeError("'convert' has to be int or string.")
        self.convert = convert

        self.frame = 0

    blueprint = "sensor.camera.rgb"

    def onData(self, data):
        super().onData(data)
        self.frame = data.frame


class CarlaRGBSensor(CarlaVisionSensor):
    def process(self, data):
        array = np.frombuffer(data.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (data.height, data.width, 4))  # BGRA format
        array = array[:, :, :3]  # Take only RGB
        array = array[:, :, ::-1]  # Revert order

        return array.copy()


class CarlaSSSensor(CarlaVisionSensor):
    blueprint = "sensor.camera.semantic_segmentation"

    def process(self, data):
        if self.convert is not None:
            data.convert(self.convert)

        array = np.frombuffer(data.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (data.height, data.width, 4))  # BGRA format

        if self.convert is not None:
            array = array[:, :, :3]  # Take only RGB
            array = array[:, :, ::-1]  # Revert order
        else:
            array = array[:, :, 2]  # Take only R

        return array.copy()
