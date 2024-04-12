""" Grid sampling (Only use for low dimensional spaces)"""

import itertools
import numpy as np
from verifai.samplers.domain_sampler import (BoxSampler, DiscreteBoxSampler,
    DomainSampler, SplitSampler, IteratorSampler, TerminationException)
from verifai.samplers.random_sampler import RandomSampler

class GridSampler(IteratorSampler):
    def __init__(self, domain, grid_params=None):
        if grid_params is None:
            grid_params = {}
        self.cont_N = grid_params.get('N', 21)
        repeat = grid_params.get('repeat', False)
        super().__init__(domain, repeat=repeat)

        cont_grid = lambda domain: ContinuousGridSampler(domain=domain,
                                                              N=self.cont_N)
        disc_grid = lambda domain: DiscreteGridSampler(domain=domain)

        partition = (
            (lambda d: d.standardizedDimension >= 0, cont_grid),
            (lambda d: d.standardizedIntervals, disc_grid)
        )
        self.split_sampler = SplitSampler.fromPartition(domain, partition)

        self.cont_sampler, self.disc_sampler = \
            self.split_sampler.samplersForPredicates

    def __iter__(self):
        while True:
            for subpoints in itertools.product(*self.split_sampler.samplers):
                yield self.domain.rejoinPoints(*subpoints)
            if not self.repeat:
                return

class ContinuousGridSampler(BoxSampler):
    def __init__(self, domain, N):
        super().__init__(domain)

        D = self.domain.standardizedDimension
        if isinstance(N, int):
            self.N = np.ones(D) * N
        else:
            if len(N) != D:
                raise RuntimeError(
                    f'grid specification for continuous space of dimension {D}'
                    f' has wrong length {len(N)}'
                )
            if not all(isinstance(k, int) and k > 0 for k in N):
                raise RuntimeError(
                    f'grid specification {N} must consist of positive integers'
                )
            self.N = N

        self.iters = 0
        self.max_iters = np.prod(self.N)

    def getVector(self):
        sample_vec = []
        t = self.iters
        if t == self.max_iters:
            self.iters = 0
            raise TerminationException('finished continuous grid sampling')

        for k in self.N:
            sample_vec.append(1./(k-1)*(t%k))
            t = t//k
        self.iters+=1
        return tuple(sample_vec), None

class DiscreteGridSampler(DiscreteBoxSampler):
    def __init__(self, domain):
        super().__init__(domain)
        self.start_N = 1
        self.iters = 0
        self.max_iters = 1
        for (left, right) in self.domain.standardizedIntervals:
            self.max_iters *= (right - left) + 1

    def getVector(self):
        if self.iters == self.max_iters:
            self.iters = 0
            raise TerminationException('finished discrete grid sampling')
        sample_vec = []
        t = self.iters
        for (left, right) in self.domain.standardizedIntervals:
            sample_vec.append(left + t%(right-left+1))
            t = t//(right-left +1)
        self.iters+=1
        return tuple(sample_vec), None
