from verifai.samplers.domain_sampler import BoxSampler
import numpy as np

class DistBayesOptSampler(BoxSampler):
    def __init__(self, domain, distBOparams):
        try:
            import GPyOpt
        except ModuleNotFoundError:
            raise RuntimeError(
                'DistBayesOptSampler requires GPyOpt to be installed')

        super().__init__(domain)
        self.num_GPs = distBOparams.num_preds
        self.func = distBOparams.func
        self.init_num = distBOparams.init_num
        if self.init_num < 1:
            raise RuntimeError(
                'init_num for DistBayesOptSampler must be at least 1')
        self.k = 10 if 'k' not in distBOparams else distBOparams.k
        self.bounds = []
        for i in range(self.dimension):
            self.bounds.append({'name': 'x_' + str(i), 'type': 'continuous',
                                'domain': (0, 1)})
        self.X = np.empty((0, self.dimension))
        self.Y = [np.empty((0, 1)) for i in range(self.num_GPs)]

    def getVector(self):
        import GPyOpt   # do this here to avoid slow import when unused
        import GPy

        if len(self.X) < self.init_num:
            # Do random sampling
            sample = np.random.uniform(0, 1, self.dimension)
        else:
            # Do BO
            X_vector = np.empty((self.num_GPs, 1))
            for i in range(self.num_GPs):
                BO = GPyOpt.methods.BayesianOptimization(f=None, batch_size=1,
                    domain=self.bounds, X=self.X, Y=self.Y[i], normalize_Y=False)
                X_vector[i] = BO.suggest_next_locations()[0]
            vals = []
            for Yi in self.Y:
                GP = GPy.models.GPRegression(self.X, Yi,
                    kernel=GPy.kern.Matern52(self.dimension))
                GP.optimize()
                m, v = GP.predict(X_vector)
                vals.append(m - self.k*v)
            func_val = self.func(vals)
            i = func_val.argmin()
            sample = X_vector[i]
        return tuple(sample), None

    def updateVector(self, vector, info, rho):
        assert len(rho) == self.num_GPs
        for i, sfb in enumerate(rho):
            self.Y[i] = np.vstack((self.Y[i], np.atleast_2d(sfb)))
