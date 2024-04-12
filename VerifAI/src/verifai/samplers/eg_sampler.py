import numpy as np
from verifai.samplers.domain_sampler import BoxSampler, DiscreteBoxSampler, \
    DomainSampler, SplitSampler
from verifai.samplers.random_sampler import RandomSampler
from verifai.samplers.cross_entropy import DiscreteCrossEntropySampler

class EpsilonGreedySampler(DomainSampler):
    def __init__(self, domain, eg_params):
        super().__init__(domain)
        self.alpha = eg_params.alpha
        self.thres = eg_params.thres
        self.cont_buckets = eg_params.cont.buckets
        self.cont_dist = eg_params.cont.dist
        self.disc_dist = eg_params.disc.dist
        self.cont_ce = lambda domain: ContinuousEpsilonGreedySampler(domain=domain,
                                                     buckets=self.cont_buckets,
                                                     dist=self.cont_dist,
                                                     alpha=self.alpha,
                                                     thres=self.thres)
        self.disc_ce = lambda domain: DiscreteEpsilonGreedySampler(domain=domain,
                                                   dist=self.disc_dist,
                                                   alpha=self.alpha,
                                                   thres=self.thres)
        partition = (
            (lambda d: d.standardizedDimension > 0, self.cont_ce),
            (lambda d: d.standardizedIntervals, self.disc_ce)
        )
        self.split_sampler = SplitSampler.fromPartition(domain,
                                                        partition,
                                                        RandomSampler)
        self.cont_sampler, self.disc_sampler = None, None
        self.rand_sampler = None
        for subsampler in self.split_sampler.samplers:
            if isinstance(subsampler, ContinuousEpsilonGreedySampler):
                assert self.cont_sampler is None
                self.cont_sampler = subsampler
            elif isinstance(subsampler, DiscreteEpsilonGreedySampler):
                assert self.disc_sampler is None
                self.disc_sampler = subsampler
            else:
                assert isinstance(subsampler, RandomSampler)
                assert self.rand_sampler is None
                self.rand_sampler = subsampler

    def getSample(self):
        return self.split_sampler.getSample()

    def update(self, sample, info, rho):
        self.split_sampler.update(sample, info, rho)

class ContinuousEpsilonGreedySampler(BoxSampler):
    def __init__(self, domain, alpha, thres,
                 buckets=10, dist=None, epsilon=0.5):
        super().__init__(domain)
        if isinstance(buckets, int):
            buckets = np.ones(self.dimension) * buckets
        elif len(buckets) > 1:
            assert len(buckets) == self.dimension
        else:
            buckets = np.ones(self.dimension) * buckets[0]
        if dist is not None:
            assert (len(dist) == len(buckets))
        if dist is None:
            dist = np.array([np.ones(int(b))/b for b in buckets])
        self.buckets = buckets
        self.dist = dist
        self.alpha = alpha
        self.thres = thres
        self.current_sample = None
        self.counts = np.array([np.ones(int(b)) for b in buckets])
        self.errors = np.array([np.zeros(int(b)) for b in buckets])
        self.t = 1
        self.epsilon = epsilon
        self.sample_randomly = np.random.uniform() < self.epsilon

    def nextVector(self, feedback=None):
        self.update(None, self.current_sample, feedback)
        return self.generateSample()
    
    def generateSample(self):
        if self.sample_randomly:
            bucket_samples = np.array([np.random.choice(int(b))
                                    for i, b in enumerate(self.buckets)])
        else:
            bucket_samples = np.array([np.random.choice(int(b), p=self.dist[i])
                                    for i, b in enumerate(self.buckets)])
        self.current_sample = bucket_samples
        ret = tuple(np.random.uniform(bs, bs+1.)/b for b, bs
              in zip(self.buckets, bucket_samples))
        return ret, bucket_samples
    
    def updateVector(self, vector, info, rho):
        if rho is None or rho >= self.thres:
            return
        self.t += 1
        if self.t % 100 == 0:
            self.epsilon *= 0.5
        update_dist = np.array([np.zeros(int(b)) for b in self.buckets])
        for i, (ud, b) in enumerate(zip(update_dist, info)):
            ud[b] = 1.
        self.dist = self.alpha*self.dist + (1-self.alpha)*update_dist
        self.sample_randomly = np.random.uniform() < self.epsilon
        # print(self.errors / self.counts)

class DiscreteEpsilonGreedySampler(DiscreteCrossEntropySampler):
    pass