from abc import abstractmethod
import math

from scenic.core.utils import sqrt2


class TrafficFlow:
    @abstractmethod
    def expected_vehs(self, stime, etime):
        pass


class ConstantTrafficFlow(TrafficFlow):
    def __init__(self, num_vehs, stime=None, etime=None):
        self.num_vehs = num_vehs
        self.stime = stime if stime is not None else 0
        self.etime = etime if stime is not None else 24 * 60 * 60
        if etime <= stime:
            raise ValueError("etime must be greater than stime.")

        self.vps = self.num_vehs / (etime - stime)

    def expected_vehs(self, stime, etime):
        if etime <= stime:
            raise ValueError("etime must be greater than stime.")

        clamped_stime = min(self.etime, max(stime, self.stime))
        clamped_etime = min(self.etime, max(etime, self.stime))

        return (clamped_etime - clamped_stime) * self.vps


class NormalTrafficFlow(TrafficFlow):
    def __init__(self, num_vehs, mean, stddev):
        self.num_vehs = num_vehs
        self.mean = mean
        self.stddev = stddev

    def expected_vehs(self, stime, etime):
        return self.num_vehs * (self.cdf(etime) - self.cdf(stime))

    def cdf(self, x):
        return (1 + math.erf((x - self.mean) / (sqrt2 * self.stddev))) / 2
