import carla
from scenic.core.sensors import ActiveSensor
import os


class CarlaVisionSensor(ActiveSensor):
    def __init__(self, offset=(0, 0, 0), rotation=(0, 0, 0), attributes=None,
                 blueprint='sensor.camera.rgb', convert=None, record=""):
        super().__init__(record=record)
        self.transform = carla.Transform(carla.Location(x=offset[0], y=offset[1], z=offset[2]),
                                         carla.Rotation(pitch=rotation[0], yaw=rotation[1], roll=rotation[2]))
        self.blueprint = blueprint
        if isinstance(attributes, str):
            raise NotImplementedError("String parsing for attributes is not yet implemented. Feel free to do so.")
        elif isinstance(attributes, dict):
            self.attributes = attributes
        else:
            self.attributes = {}

        self.convert = None
        if convert is not None:
            if isinstance(convert, int):
                self.convert = convert
            elif isinstance(convert, str):
                self.convert = carla.ColorConverter.names[convert]
            else:
                AttributeError("'convert' has to be int or string.")

    def record_last_observation(self, record_path):
        self.last_observation.save_to_disk(os.path.join(record_path, f"frame_{self.last_observation.frame}.jpg"))

    def processing(self, data):
        if self.convert is not None:
            data.convert(self.convert)
            return data
        return data


class CarlaRGBSensor(CarlaVisionSensor):
    def __init__(self, offset=(0, 0, 0), rotation=(0, 0, 0), attributes=None, convert=None, record=""):
        super().__init__(offset=offset, rotation=rotation, attributes=attributes,
                         blueprint='sensor.camera.rgb', convert=convert, record=record)


class CarlaSSSensor(CarlaVisionSensor):
    def __init__(self, offset=(0, 0, 0), rotation=(0, 0, 0), attributes=None, convert=None, record=""):
        super().__init__(offset=offset, rotation=rotation, attributes=attributes,
                         blueprint='sensor.camera.semantic_segmentation', convert=convert, record=record)
