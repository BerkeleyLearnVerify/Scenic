"""Rejection sampling"""

from verifai.utils.utils import RejectionException
from verifai.samplers.domain_sampler import ConstrainedSampler, SamplingError

class RejectionSampler(ConstrainedSampler):
    """Enforces a spec over some other sampler by rejection.

    RejectionSampler(RandomSampler(domain), spec)
    """

    def __init__(self, sampler, spec=None, rejectionRho=None, maxRejections=1000):
        super().__init__(sampler.domain, spec)
        self.sampler = sampler
        self.specification = spec
        self.rejectionRho = None
        self.maxRejections = maxRejections

    def getSample(self):
        reject = True
        samples = 0
        while reject:
            if samples >= self.maxRejections:
                raise SamplingError(
                    f'exceeded RejectionSampler limit of {samples} rejections')
            samples += 1
            try:
                sample, info = self.sampler.getSample()
                if self.specification is not None:
                    reject = not self.specification.isSatisfiedBy(sample)
                else:
                    reject = False
                if reject:
                    self.sampler.update(sample, info, self.rejectionRho)
            except RejectionException:
                reject = True
        return sample, info

    def update(self, sample, info, rho):
        self.sampler.update(sample, info, rho)

    def __repr__(self):
        return (f'RejectionSampler({self.sampler}, spec={self.specification}, '
                f'maxRejections={self.maxRejections})')
