from verifai.features import *
from verifai.samplers import *
from dotmap import DotMap
from verifai.samplers.dist_BO import DistBayesOptSampler
from tests.utils import sampleWithFeedback

import pytest

pytest.importorskip('GPyOpt')


def test_distBO():
    x_domain = FeatureSpace({'x': Feature(Box([-np.pi, np.pi]))})

    def preds(sample):
        print(sample)
        return [np.sin(sample.x[0]), np.cos(sample.x[0])]

    def func(vals):
        print(vals)
        return np.maximum(vals[0], vals[1])

    distBOparams = DotMap()
    distBOparams.func = func
    distBOparams.init_num= 10
    distBOparams.num_preds = 2

    makeDomainSampler = (lambda domain:
        DistBayesOptSampler(domain=domain, distBOparams=distBOparams))
    sampler = LateFeatureSampler(x_domain, RandomSampler, makeDomainSampler)

    sampleWithFeedback(sampler, 15, preds)
