from verifai.features import *
from verifai.samplers import *
from dotmap import DotMap

import pytest
from tests.utils import sampleWithFeedback, checkSaveRestore

pytest.importorskip('GPyOpt')

def test_bayesianOptimization():
    carDomain = Struct({
        'position': Box([-10,10], [-10,10], [0,1]),
        'heading': Box([0, math.pi]),
    })

    space = FeatureSpace({
        'cars': Feature(Array(carDomain, [2]))
    })

    def f(sample):
        sample = sample.cars[0].heading[0]
        return abs(sample - 0.75)

    bo_params = DotMap()
    bo_params.init_num = 2

    sampler = FeatureSampler.bayesianOptimizationSamplerFor(space, BO_params=bo_params)

    sampleWithFeedback(sampler, 3, f)

def test_save_restore(tmpdir):
    space = FeatureSpace({
        'a': Feature(DiscreteBox([0, 12])),
        'b': Feature(Box((0, 1)), lengthDomain=DiscreteBox((1, 2)))
    })
    bo_params = DotMap(init_num=2)
    sampler = FeatureSampler.bayesianOptimizationSamplerFor(space, bo_params)

    checkSaveRestore(sampler, tmpdir)

def test_BO_oper():
    vals = []
    N = 10
    for k in range(N):
        print("Testing k: ", k)
        def f(sample):
            x = sample.x[0]
            return (6 * x - 2) ** 2 * np.sin(12 * x - 4)

        space = FeatureSpace({'x': Feature(Box([0,1]))})

        bo_params = DotMap()
        bo_params.init_num = 5

        sampler = FeatureSampler.bayesianOptimizationSamplerFor(space, BO_params=bo_params)

        samples = []
        y_samples = []

        feedback = None
        for i in range(20):
            sample = sampler.nextSample(feedback)
            samples.append(sample)
            feedback = f(sample)
            y_samples.append(feedback)

        min_i = np.array(y_samples).argmin()
        vals.append(np.linalg.norm(samples[min_i].x[0]-0.75) < 0.1)

    assert sum(vals)/N >= 0.9


