
import random

import pytest

from tests.utils import compileScenic, sampleEgo, sampleParamP

# Skip tests if verifai not installed
# (to try to catch errors inside verifai, don't use "pytest.importorskip")
try:
    import verifai
except ModuleNotFoundError:
    pytest.skip('verifai package not installed', allow_module_level=True)

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

def checkCEConvergence(scenario, rangeCheck=(lambda x: x == -1 or x == 1)):
    f = lambda ego: -1 if ego.position.x > 0 else 1
    xs = [ego.position.x for ego in sampleEgoWithFeedback(scenario, f, 1200)]
    assert all(rangeCheck(x) for x in xs)
    assert 22 <= sum(x < 0 for x in xs[:30])
    assert 143 <= sum(x > 0 for x in xs[200:])

## Particular samplers

def test_halton():
    scenario = compileScenic(
        'param verifaiSamplerType = "halton"\n'
        'ego = Object at VerifaiRange(5, 15) @ 0'
    )
    xs = [sampleEgo(scenario).position.x for i in range(60)]
    assert all(5 <= x <= 15 for x in xs)
    assert 29 <= sum(x < 10 for x in xs) <= 31

def test_cross_entropy():
    scenario = compileScenic(
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

def test_cross_entropy_inline():
    scenario = compileScenic(
        'param verifaiSamplerType = "ce"\n'
        'from dotmap import DotMap\n'
        'param verifaiSamplerParams = DotMap(alpha=0.99)\n'
        'ego = Object at VerifaiRange(-1, 1, weights=[100, 1]) @ 0'
    )
    checkCEConvergence(scenario, rangeCheck=(lambda x: -1 <= x <= 1))

def test_cross_entropy_options():
    scenario = compileScenic(
        'param verifaiSamplerType = "ce"\n'
        'from dotmap import DotMap\n'
        'param verifaiSamplerParams = DotMap(alpha=0.99)\n'
        'ego = Object at VerifaiOptions({-1: 100, 1: 1}) @ 0'
    )
    checkCEConvergence(scenario)

def test_cross_entropy_prior():
    scenario = compileScenic(
        'param verifaiSamplerType = "ce"\n'
        'from dotmap import DotMap\n'
        'param verifaiSamplerParams = DotMap(alpha=0.99)\n'
        'ego = Object at VerifaiParameter.withPrior(Options({-1: 100, 1: 1})) @ 0'
    )
    checkCEConvergence(scenario)

def test_cross_entropy_prior_normal():
    scenario = compileScenic(
        'param verifaiSamplerType = "ce"\n'
        'from dotmap import DotMap\n'
        'param verifaiSamplerParams = DotMap(alpha=0.99)\n'
        'ego = Object at VerifaiParameter.withPrior(Normal(-1, 0.7)) @ 0'
    )
    checkCEConvergence(scenario, rangeCheck=(lambda x: True))

## Reproducibility and noninterference

def test_reproducibility():
    scenario = compileScenic(
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

def test_noninterference():
    for i in range(2):
        scenario = compileScenic('ego = Object at VerifaiRange(0, 1) @ 0')
        for j in range(5):
            scene, iterations = scenario.generate(maxIterations=1)
            assert len(scenario.externalSampler.cachedSample) == 1
