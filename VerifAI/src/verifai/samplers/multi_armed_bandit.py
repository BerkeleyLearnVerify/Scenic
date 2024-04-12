import numpy as np
import networkx as nx
from itertools import product
from verifai.samplers.domain_sampler import BoxSampler, DiscreteBoxSampler, \
    DomainSampler, SplitSampler
from verifai.samplers.random_sampler import RandomSampler
from verifai.samplers.cross_entropy import DiscreteCrossEntropySampler
from verifai.samplers.multi_objective import MultiObjectiveSampler

class MultiArmedBanditSampler(DomainSampler):
    def __init__(self, domain, mab_params):
        super().__init__(domain)
        self.alpha = mab_params.alpha
        self.thres = mab_params.thres
        self.cont_buckets = mab_params.cont.buckets
        self.cont_dist = mab_params.cont.dist
        self.disc_dist = mab_params.disc.dist
        self.cont_ce = lambda domain: ContinuousMultiArmedBanditSampler(domain=domain,
                                                     buckets=self.cont_buckets,
                                                     dist=self.cont_dist,
                                                     alpha=self.alpha,
                                                     thres=self.thres)
        self.disc_ce = lambda domain: DiscreteMultiArmedBanditSampler(domain=domain,
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
            if isinstance(subsampler, ContinuousMultiArmedBanditSampler):
                assert self.cont_sampler is None
                if 'priority_graph' in mab_params:
                    subsampler.set_graph(mab_params.priority_graph)
                self.cont_sampler = subsampler
            elif isinstance(subsampler, DiscreteMultiArmedBanditSampler):
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

class ContinuousMultiArmedBanditSampler(BoxSampler, MultiObjectiveSampler):
    def __init__(self, domain, alpha, thres,
                 buckets=10, dist=None, restart_every=100):
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
        self.counterexamples = dict()
        self.is_multi = False
        self.invalid = np.array([np.zeros(int(b)) for b in buckets])
        self.monitor = None
        self.rho_values = []
        self.restart_every = restart_every

    def getVector(self):
        return self.generateSample()
    
    def generateSample(self):
        proportions = self.errors / self.counts
        Q = proportions + np.sqrt(2 / self.counts * np.log(self.t))
        # choose the bucket with the highest "goodness" value, breaking ties randomly.
        bucket_samples = np.array([np.random.choice(np.flatnonzero(np.isclose(Q[i], Q[i].max())))
            for i in range(len(self.buckets))])
        self.current_sample = bucket_samples
        ret = tuple(np.random.uniform(bs, bs+1.)/b for b, bs
              in zip(self.buckets, bucket_samples))
        return ret, bucket_samples
    
    def updateVector(self, vector, info, rho):
        assert rho is not None
        self.t += 1
        # "random restarts" to generate a new topological sort of the priority graph
        # every restart_every samples.
        if self.is_multi:
            if self.monitor is not None and self.monitor.linearize and t % self.restart_every == 0:
                self.monitor._linearize()
            self.update_dist_from_multi(vector, info, rho)
            return
        for i, b in enumerate(info):
            self.counts[i][b] += 1.
            if rho < self.thres:
                self.errors[i][b] += 1.

    # is rho1 better than rho2?
    # partial pre-ordering on objective functions, so it is possible that:
    # is_better_counterexample(rho1, rho2)
    # and is_better_counterxample(rho2, rho1) both return False
    def is_better_counterexample(self, ce1, ce2):
        if ce2 is None:
            return True
        all_same = True
        already_better = [False] * self.num_properties
        for node in nx.dfs_preorder_nodes(self.priority_graph):
            if already_better[node]:
                continue
            b1 = ce1[node]
            b2 = ce2[node]
            all_same = all_same and b1 == b2
            if b2 and not b1:
                return False
            if b1 and not b2:
                already_better[node] = True
                for subnode in nx.descendants(self.priority_graph, node):
                    already_better[subnode] = True
        return not all_same

    def _get_total_counterexamples(self):
        return sum(self.counterexamples.values())
    
    @property
    def counterexample_values(self):
        return [ce in self.counterexamples for ce in self.rho_values]

    def _add_to_running(self, ce):
        if ce in self.counterexamples:
            return True
        to_remove = set()
        # if there is already a better counterexample, don't add this.
        if len(self.counterexamples) > 0:
            for other_ce in self.counterexamples:
                if self.is_better_counterexample(other_ce, ce):
                    return False
        # remove all worse counterexamples than this.
        for other_ce in self.counterexamples:
            if self.is_better_counterexample(ce, other_ce):
                to_remove.add(other_ce)
        for other_ce in to_remove:
            del self.counterexamples[other_ce]
        self.counterexamples[ce] = np.array([np.zeros(int(b)) for b in self.buckets])
        return True
    
    def update_dist_from_multi(self, sample, info, rho):
        try:
            iter(rho)
        except:
            for i, b in enumerate(info):
                self.invalid[i][b] += 1.
            return
        if len(rho) != self.num_properties:
            for i, b in enumerate(info):
                self.invalid[i][b] += 1.
            return
        # print('inside update_dist_from_multi')
        counter_ex = tuple(
            rho[node] < self.thres[node] for node in nx.dfs_preorder_nodes(self.priority_graph)
        )
        self.rho_values.append(counter_ex)
        # print(f'counter_ex = {counter_ex}')
        # print(self.counterexamples)
        is_ce = self._add_to_running(counter_ex)
        for i, b in enumerate(info):
            self.counts[i][b] += 1.
            if is_ce:
                self.counterexamples[counter_ex][i][b] += 1.
        self.errors = self.invalid + self._get_total_counterexamples()

class DiscreteMultiArmedBanditSampler(DiscreteCrossEntropySampler):
    pass