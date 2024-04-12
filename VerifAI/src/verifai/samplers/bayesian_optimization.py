"""Bayesian Optimization sampler : Defined only for continuous domains.
For discrete inputs define another sampler"""

from verifai.samplers.domain_sampler import BoxSampler
import numpy as np


class BayesOptSampler(BoxSampler):
    def __init__(self, domain, BO_params):
        try:
            import GPyOpt
        except ModuleNotFoundError:
            raise RuntimeError(
                'BayesOptSampler requires GPyOpt to be installed')

        super().__init__(domain)
        self.init_num = BO_params.init_num
        if self.init_num < 1:
            raise RuntimeError(
                'init_num for BayesOptSampler must be at least 1')
        self.bounds = []
        for i in range(self.dimension):
            self.bounds.append({'name':'x_'+str(i), 'type': 'continuous',
                                'domain': (0,1)})
        self.X = np.empty((0, self.dimension))
        self.Y = np.empty((0, 1))

    def getVector(self, feedback=None):
        import GPyOpt   # do this here to avoid slow import when unused

        if len(self.X) < self.init_num:
            # Do random sampling
            sample = np.random.uniform(0, 1, self.dimension)
        else:
            BO = GPyOpt.methods.BayesianOptimization(
                f=None, batch_size=1,
                domain=self.bounds, X=self.X, Y=self.Y, normalize_Y=False)
            sample = BO.suggest_next_locations()[0]
        return tuple(sample), None

    def updateVector(self, vector, info, rho):
        self.X = np.vstack((self.X, np.atleast_2d(vector)))
        self.Y = np.vstack((self.Y, np.atleast_2d(rho)))
