class Sensor:
    def get_last_observation(self):
        raise NotImplementedError("Not implemented for passive sensors")

    def save_last_observation(self, save_path, filename=""):
        raise NotImplementedError("Implement in subclass")


class ActiveSensor(Sensor):

    def __init__(self):
        self.last_observation = None

    def get_last_observation(self):
        return self.last_observation

    def on_data(self, data):
        self.last_observation = self.processing(data)

    def processing(self, data):
        return data

    def save_last_observation(self, save_path, filename=""):
        raise NotImplementedError("Implement in subclass")
