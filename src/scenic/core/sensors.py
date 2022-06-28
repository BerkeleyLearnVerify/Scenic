import os


class Sensor:
    def get_last_observation(self):
        raise NotImplementedError("Not implemented for passive sensors")

    def record_last_observation(self, record_path):
        raise NotImplementedError("Implement in subclass")


class ActiveSensor(Sensor):
    last_observation: None

    def __init__(self, record=""):
        self.record_path = record
        if record:
            os.makedirs(record, exist_ok=True)

    def get_last_observation(self):
        return self.last_observation

    def on_data(self, data):
        self.last_observation = self.processing(data)
        if self.record_path:
            self.record_last_observation(self.record_path)

    def processing(self, data):
        return data

    def record_last_observation(self, record_path):
        raise NotImplementedError("Implement in subclass")
