from dotmap import DotMap

from verifai.features import *
from verifai.samplers import *
from tests.utils import checkSaveRestore

def test_halton():
    carDomain = Struct({
        'position': Box([-10,10], [-10,10], [0,1]),
        'heading': Box([0, math.pi]),
        'model': DiscreteBox([0, 10])
    })

    space = FeatureSpace({
        'weather': Feature(DiscreteBox([0,12])),
        'cars': Feature(Array(carDomain, [2]))
    })

    halton_params = DotMap()
    halton_params.sample_index = -2
    halton_params.bases_skipped = 0

    sampler = FeatureSampler.haltonSamplerFor(space, halton_params)

    for i in range(3):
        print(f'Sample #{i}:')
        print(sampler.nextSample())

def test_save_restore(tmpdir):
    space = FeatureSpace({
        'a': Feature(DiscreteBox([0, 12])),
        'b': Feature(Box((0, 1)), lengthDomain=DiscreteBox((1, 2)))
    })
    halton_params = DotMap(sample_index=0, bases_skipped=0)
    sampler = FeatureSampler.haltonSamplerFor(space, halton_params)

    checkSaveRestore(sampler, tmpdir)
