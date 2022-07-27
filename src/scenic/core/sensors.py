import os


class Sensor:
    def get_last_observation(self):
        raise NotImplementedError("Not implemented for passive sensors")

    def save_last_observation(self, save_path, filename=""):
        raise NotImplementedError("Implement in subclass")


class ActiveSensor(Sensor):
    last_observation: None

    def __init__(self, save_path=""):
        self.save_path = save_path
        if save_path:
            os.makedirs(save_path, exist_ok=True)

    def get_last_observation(self):
        return self.last_observation

    def on_data(self, data):
        self.last_observation = self.processing(data)
        if self.save_path:
            self.save_last_observation(self.save_path)

    def processing(self, data):
        return data

    def save_last_observation(self, save_path, filename=""):
        raise NotImplementedError("Implement in subclass")
