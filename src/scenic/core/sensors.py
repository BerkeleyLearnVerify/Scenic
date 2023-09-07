import copy


class Sensor:
    def get_observation(self):
        raise NotImplementedError("Not implemented for passive sensors")

    def save_last_observation(self, save_path, frame_number=None):
        raise NotImplementedError("Implement in subclass")


class ActiveSensor(Sensor):

    def __init__(self):
        self.observation = None
        self.frame = 0

    def get_observation(self):
        return self.observation

    def on_data(self, data):
        print("On data", self.__class__)
        print("Frame", data.frame)
        self.observation, self.frame = self.processing(data)

    def processing(self, data):
        raise NotImplementedError("Implement in subclass")

    def save_last_observation(self, save_path, frame_number=None):
        raise NotImplementedError("Implement in subclass")
