
import random

import pytest

from tests.utils import compileScenic, sampleEgo, sampleParamP

# Skip tests if verifai not installed
pytest.importorskip('verifai')

## Utilities

def sampleEgoWithFeedback(scenario, f, numSamples, maxIterations=1):
    feedback = None
    egos = []
    for i in range(numSamples):
        scene, _ = scenario.generate(maxIterations=maxIterations, feedback=feedback)
        ego = scene.egoObject
        egos.append(ego)
        feedback = f(ego)
    return egos

## Particular samplers

def test_halton():
    scenario = compileScenic(
        'from scenic.core.external_params import *\n'
        'param verifaiSamplerType = "halton"\n'
        'ego = Object at VerifaiRange(5, 15) @ 0'
    )
    xs = [sampleEgo(scenario).position.x for i in range(60)]
    assert all(5 <= x <= 15 for x in xs)
    assert 29 <= sum(x < 10 for x in xs) <= 31

def test_cross_entropy():
    scenario = compileScenic(
        'from scenic.core.external_params import *\n'
        'param verifaiSamplerType = "ce"\n'
        'from dotmap import DotMap\n'
        'ce_params = DotMap()\n'
        'ce_params.alpha = 0.9\n'
        'ce_params.cont.buckets = 2\n'
        'param verifaiSamplerParams = ce_params\n'
        'ego = Object at VerifaiRange(5, 15) @ 0'
    )
    f = lambda ego: -1 if ego.position.x < 10 else 1
    xs = [ego.position.x for ego in sampleEgoWithFeedback(scenario, f, 120)]
    assert all(5 <= x <= 15 for x in xs)
    assert any(x > 10 for x in xs)
    assert 66 <= sum(x < 10 for x in xs[50:])

## Reproducibility

def test_reproducibility():
    scenario = compileScenic(
        'from scenic.core.external_params import *\n'
        'param verifaiSamplerType = "ce"\n'
        'ego = Object at VerifaiRange(-1, 1) @ 0'
    )
    import numpy
    f = lambda ego: -1 if ego.position.x < 10 else 1
    def sampleSequence(seed):
        random.seed(seed)
        numpy.random.seed(seed)    # TODO improve?
        scenario.resetExternalSampler()
        return [ego.position.x for ego in sampleEgoWithFeedback(scenario, f, 10)]

    seeds = [random.randint(0, 100000) for i in range(10)]
    for seed in seeds:
        base = sampleSequence(seed)
        other = sampleSequence(seed)
        assert base == other
