from abc import ABC, abstractmethod

class MultiObjectiveSampler:

    def set_priority_graph(self, graph):
        self.priority_graph = graph

    """
    Update distribution from list of objectives.

    Arguments:
        sample: the sample as outputted by ScenicSampler.nextSample()
        info: Contains any necessary information to perform the update (such as bucket indices, etc.)
        rho: A NumPy array with the real objective values [f1, f2, f3, ..., fn]
    """
    @abstractmethod
    def update_dist_from_multi(self, sample, info, rho):
        pass
