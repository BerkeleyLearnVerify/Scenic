from verifai.features import *
from verifai.samplers import *
from collections import defaultdict

def test_grid():
    space = FeatureSpace({
        'weather': Feature(DiscreteBox([0,12])),
        'car_positions': Feature(Box([-10,10], [0,1]))
    })

    sampler = FeatureSampler.gridSamplerFor(space)

    i = 0
    dict_samples = defaultdict(int)
    while True:
        try:
            sample = sampler.nextSample()
            dict_samples[(sample.weather[0], sample.car_positions[0],
                          sample.car_positions[1])] = 0
        except TerminationException:
            break
        i+=1
    assert i == len(dict_samples) and i == 21*21*13

def test_grid_iter():
    space = FeatureSpace({
        'weather': Feature(DiscreteBox([0,12])),
        'day': Feature(DiscreteBox([1,7])),
        'car_positions': Feature(Box([-10,10]))
    })

    sampler = FeatureSampler.gridSamplerFor(space)
    samples = list(sampler)
    assert len(samples) == 13*7*21

def test_grid_oper():
    def f(sample):
        x = sample.x[0]
        return (6 * x - 2) ** 2 * np.sin(12 * x - 4)

    space = FeatureSpace({'x': Feature(Box([0,1]))})

    sampler = FeatureSampler.gridSamplerFor(space)

    samples = []
    y_samples = []

    for i in range(21):
        sample = sampler.nextSample()
        samples.append(sample)
        y_samples.append(f(sample))

    min_i = np.array(y_samples).argmin()
    assert min_i == 15

def test_grid_non_standardizable():
    space = FeatureSpace({
        'a': Feature(DiscreteBox([0,12])),
        'b': Feature(FilteredDomain(Box([0,1]), lambda x: x[0] > 0.5))
    })
    sampler = FeatureSampler.gridSamplerFor(space)
    samples = list(sampler)
    assert len(samples) == 13
    assert all(sample.b[0] > 0.5 for sample in samples)
