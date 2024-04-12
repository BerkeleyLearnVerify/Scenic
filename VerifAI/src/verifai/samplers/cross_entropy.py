"""Cross-entropy samplers"""

import numpy as np
from verifai.samplers.domain_sampler import BoxSampler, DiscreteBoxSampler, \
    DomainSampler, SplitSampler
from verifai.samplers.random_sampler import RandomSampler
import networkx as nx

class CrossEntropySampler(DomainSampler):
    def __init__(self, domain, ce_params):
        super().__init__(domain)
        self.alpha = ce_params.alpha
        self.thres = ce_params.thres
        self.cont_buckets = ce_params.cont.buckets
        self.cont_dist = ce_params.cont.dist
        self.disc_dist = ce_params.disc.dist
        self.cont_ce = lambda domain: ContinuousCrossEntropySampler(domain=domain,
                                                     buckets=self.cont_buckets,
                                                     dist=self.cont_dist,
                                                     alpha=self.alpha,
                                                     thres=self.thres)
        self.disc_ce = lambda domain: DiscreteCrossEntropySampler(domain=domain,
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
            if isinstance(subsampler, ContinuousCrossEntropySampler):
                assert self.cont_sampler is None
                self.cont_sampler = subsampler
            elif isinstance(subsampler, DiscreteCrossEntropySampler):
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

class ContinuousCrossEntropySampler(BoxSampler):
    def __init__(self, domain, alpha, thres,
                 buckets=10, dist=None):
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
            dist = [np.ones(int(b))/b for b in buckets]
        self.buckets = buckets
        self.dist = dist
        self.alpha = alpha
        self.thres = thres
        self.current_sample = None

    def getVector(self):
        bucket_samples = np.array([np.random.choice(int(b), p=self.dist[i])
                                   for i, b in enumerate(self.buckets)])
        self.current_sample = bucket_samples
        ret = tuple(np.random.uniform(bs, bs+1.)/b for b, bs
              in zip(self.buckets, bucket_samples))
        return ret, bucket_samples

    def updateVector(self, vector, info, rho):
        if rho is None or rho >= self.thres:
            return
        for row, b in zip(self.dist, info):
            row *= self.alpha
            row[b] += 1 - self.alpha

class DiscreteCrossEntropySampler(DiscreteBoxSampler):
    def __init__(self, domain, alpha, thres, dist=None):
        super().__init__(domain)
        if dist is not None:
            assert (len(dist) == len(domain.standardizedIntervals))
        if dist is None:
            dist = [np.ones(right-left+1)/(right-left+1)
                    for left, right in domain.standardizedIntervals]
        self.dist = dist
        self.alpha = alpha
        self.thres = thres
        self.current_sample = None

    def getVector(self):
        self.current_sample=\
            tuple(left + np.random.choice(right-left+1, p=self.dist[i])
                     for i, (left, right) in enumerate(self.domain.standardizedIntervals))
        return self.current_sample, None

    def updateVector(self, vector, info, rho):
        assert rho is not None
        if rho >= self.thres:
            return
        for row, (left, right), b in zip(self.dist, self.domain.standardizedIntervals, vector):
            row *= self.alpha
            row[b-left] += 1 - self.alpha

class MultiContinuousCrossEntropySampler(ContinuousCrossEntropySampler):
    
    def __init__(self, domain, alpha, thres, priority_graph=None,
                 buckets=10, dist=None, epsilon=0.5):
        if priority_graph is not None:
            self.set_graph(priority_graph)
        self.epsilon = epsilon
        self.still_sampling = False
        super().__init__(domain, alpha, thres, buckets=10, dist=dist)
        self.counts = np.array([np.zeros(int(b)) for b in self.buckets])

    def getVector(self):
        if not self.still_sampling:
            self.sample_randomly = np.random.uniform() < self.epsilon
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

    def set_graph(self, graph):
        self.priority_graph = graph
        if graph is not None:
            self.thres = [self.thres] * graph.number_of_nodes()
            self.num_properties = graph.number_of_nodes()

    def updateVector(self, vector, info, rho):
        if isinstance(rho, int):
            self.still_sampling = True
            return
        if rho is None or any([r is None for r in rho]):
            self.still_sampling = False
            return
        to_update = [True] * self.num_properties
        for node in nx.dfs_preorder_nodes(self.priority_graph):
            if not to_update[node]:
                continue
            b = rho[node] <= self.thres[node]
            if not b:
                to_update[node] = False
                for subnode in nx.descendants(self.priority_graph, node):
                    to_update[subnode] = False
        num_updates = sum(to_update)
        for crow, drow, b in zip(self.counts, self.dist, info):
            crow[b] += 1
            for _ in range(num_updates):
                row *= self.alpha
                row[b] += 1 - self.alpha
