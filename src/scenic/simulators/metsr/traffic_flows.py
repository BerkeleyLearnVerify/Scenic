from abc import abstractmethod


class TrafficFlow:
    @abstractmethod
    def vps(self, time):
        pass

    def probSpawn(self, time, timestep):
        return self.vps(time) * timestep


class ConstantTrafficFlow(TrafficFlow):
    def __init__(self, vph):
        self.vph = vph

    def vps(self, time):
        return self.vph / (60 * 60)
