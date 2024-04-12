from verifai.features.features import *
from verifai.samplers.feature_sampler import *

def test_example():
    carDomain = Struct({
        'position': Box([-10,10], [-10,10], [0,1]),
        'heading': Box([0, math.pi])
    })

    space = FeatureSpace({
        'weather': Feature(DiscreteBox([0,12])),
        'cars': Feature(carDomain, lengthDomain=DiscreteBox([0,2]))
    })

    sampler = FeatureSampler.samplerFor(space)

    for i in range(3):
        sample = sampler.nextSample()
        print(f'Sample #{i}:')
        print(sample)
        flat = space.flatten(sample)
        unflat = space.unflatten(flat)
        assert sample == unflat
