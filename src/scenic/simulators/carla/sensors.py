import carla
import numpy as np

from scenic.core.sensors import ActiveSensor, Sensor
import os


class CarlaVisionSensor(ActiveSensor):
    def __init__(self, offset=(0, 0, 0), rotation=(0, 0, 0), attributes=None,
                 blueprint='sensor.camera.rgb', convert=None, save_path="", record_npy=True):
        super().__init__(save_path=save_path)
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

        self.record_npy = record_npy

    def processing(self, data):
        if self.convert is not None:
            data.convert(self.convert)
            return data
        return data


class CarlaRGBSensor(CarlaVisionSensor):
    def __init__(self, offset=(0, 0, 0), rotation=(0, 0, 0), attributes=None, convert=None, save_path="",
                 record_npy=True):
        super().__init__(offset=offset, rotation=rotation, attributes=attributes,
                         blueprint='sensor.camera.rgb', convert=convert, save_path=save_path, record_npy=record_npy)

    def save_last_observation(self, save_path, filename=""):
        if not filename:
            filename = f"frame_{self.last_observation.frame}"
        save_as = os.path.join(save_path, filename)
        if self.record_npy:
            array = np.frombuffer(self.last_observation.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (self.last_observation.height, self.last_observation.width, 4))  # RGBA format
            array = array[:, :, :3]  # Take only RGB
            array = array[:, :, ::-1]  # Revert order
            np.save(save_as, array)
        else:
            self.last_observation.save_to_disk(save_as)


class CarlaSSSensor(CarlaVisionSensor):
    def __init__(self, offset=(0, 0, 0), rotation=(0, 0, 0), attributes=None, convert=None, save_path="",
                 record_npy=True):
        super().__init__(offset=offset, rotation=rotation, attributes=attributes,
                         blueprint='sensor.camera.semantic_segmentation', convert=convert, save_path=save_path,
                         record_npy=record_npy)

    def save_last_observation(self, save_path, filename=""):
        if not filename:
            filename = f"frame_{self.last_observation.frame}"
        save_as = os.path.join(save_path, filename)
        if self.record_npy:
            array = np.frombuffer(self.last_observation.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (self.last_observation.height, self.last_observation.width, 4))  # RGBA format
            array = array[:, :, 2]  # Take only R
            np.save(save_as, array)
        else:
            self.last_observation.save_to_disk(save_as)


