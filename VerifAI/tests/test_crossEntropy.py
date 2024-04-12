from verifai.features import *
from verifai.samplers import *
from dotmap import DotMap

from tests.utils import sampleWithFeedback, checkSaveRestore

def test_crossEntropy():
    carDomain = Struct({
        'position': Box([-10,10], [-10,10], [0,1]),
        'heading': Box([0, math.pi]),
        'model': DiscreteBox([0, 10])
    })

    space = FeatureSpace({
        'weather': Feature(DiscreteBox([0,12])),
        'cars': Feature(Array(carDomain, [2]))
    })

    def f(sample):
        print(sample.cars[0].heading[0] - 0.75)
        return abs(sample.cars[0].heading[0] - 0.75)

    ce_params = DotMap()
    ce_params.alpha =0.9
    ce_params.thres = 0.25
    ce_params.cont.buckets = 5
    ce_params.cont.dist= None
    ce_params.disc.dist = None

    sampler = FeatureSampler.crossEntropySamplerFor(space, ce_params=ce_params)

    sampleWithFeedback(sampler, 100, f)

def test_feedback_multiple_lengths():
    space = FeatureSpace({
        'a': Feature(Box((0, 1)), lengthDomain=DiscreteBox((1, 2)))
    })

    def f(sample):
        assert 1 <= len(sample.a) <= 2
        return -1 if len(sample.a) == 1 and sample.a[0][0] < 0.5 else 1

    ce_params = DotMap(alpha=0.5, thres=0)
    ce_params.cont.buckets = 2
    ce_params.cont.dist = None
    ce_params.disc.dist = None
    sampler = FeatureSampler.crossEntropySamplerFor(space, ce_params)

    sampleWithFeedback(sampler, 100, f)
    l1sampler = sampler.domainSamplers[sampler.lengthDomain.makePoint(a=(1,))]
    l1dist = l1sampler.cont_sampler.dist[0]
    l2sampler = sampler.domainSamplers[sampler.lengthDomain.makePoint(a=(2,))]
    l2dists = l2sampler.cont_sampler.dist
    assert len(l1dist) == 2
    assert l1dist[0] > 0.9
    assert all(list(l2dist) == [0.5, 0.5] for l2dist in l2dists)

def test_save_restore(tmpdir):
    space = FeatureSpace({
        'a': Feature(DiscreteBox([0, 12])),
        'b': Feature(Box((0, 1))),
        'c': Feature(Box((0, 1), (-1, 1)), lengthDomain=DiscreteBox((0, 1)))
    })
    ce_params = DotMap(alpha=0.9, thres=0)
    ce_params.cont.buckets = 5
    ce_params.cont.dist = None
    ce_params.disc.dist = None
    sampler = FeatureSampler.crossEntropySamplerFor(space, ce_params)

    checkSaveRestore(sampler, tmpdir, iterations=30)
