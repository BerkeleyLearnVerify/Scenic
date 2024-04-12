"""Random sampling"""

from verifai.samplers.domain_sampler import DomainSampler

class RandomSampler(DomainSampler):
    """Samples uniformly at random or from a given distribution"""

    def __init__(self, domain, distribution=None):
        super().__init__(domain)
        self.distribution = distribution

    def getSample(self):
        if self.distribution is None:
            return self.domain.uniformPoint(), None
        else:
            return self.distribution.sample(), None

    def __repr__(self):
        rep = f'RandomSampler({self.domain}'
        if self.distribution is not None:
            rep += f', distribution={self.distribution}'
        return rep + ')'
