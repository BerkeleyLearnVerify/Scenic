import random

import numpy as np
import pytest

from tests.utils import compileScenic, sampleEgo, sampleParamP

# Skip tests if verifai not installed
# (to try to catch errors inside verifai, don't use "pytest.importorskip")
try:
    import verifai
except ModuleNotFoundError:
    pytest.skip("verifai package not installed", allow_module_level=True)

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


@pytest.fixture
def checkCEConvergence(pytestconfig):
    fast = pytestconfig.getoption("--fast")

    def helper(*args, **kwargs):
        if "fast" not in kwargs:
            kwargs["fast"] = fast
        return _ceHelper(*args, **kwargs)

    return helper


def _ceHelper(scenario, rangeCheck=None, warmup=0, fast=False):
    if not rangeCheck:
        rangeCheck = lambda x: x == -1 or x == 1
    f = lambda ego: -1 if ego.position.x > 0 else 1
    if fast:
        iterations = 120
    else:
        iterations = 1250 + warmup
    xs = [ego.position.x for ego in sampleEgoWithFeedback(scenario, f, iterations)]
    assert all(rangeCheck(x) for x in xs)
    assert any(x <= 0 for x in xs[:120])
    if fast:
        return
    # The following parameters are for alpha=0.99, 1250 iterations, 2 buckets
    # and all the counterexamples in the x > 0 bucket. With a uniform prior, no
    # warmup is needed; for 9:1 odds against the right bucket, use warmup=1300.
    xs = xs[warmup:]
    assert 58 <= sum(x > 0 for x in xs[:250])
    assert 590 <= sum(x > 0 for x in xs[250:])
    assert 195 <= sum(x > 0 for x in xs[1000:])


def checkCEStationary(scenario):
    f = lambda ego: 1
    xs = [ego.position.x for ego in sampleEgoWithFeedback(scenario, f, 250)]
    # These parameters assume 2 buckets with 9:1 odds against the positive bucket
    assert 175 <= sum(x <= 0 for x in xs) < 250


## Particular samplers


def test_halton():
    scenario = compileScenic(
        'param verifaiSamplerType = "halton"\n'
        "ego = new Object at VerifaiRange(5, 15) @ 0"
    )
    xs = [sampleEgo(scenario).position.x for i in range(60)]
    assert all(5 <= x <= 15 for x in xs)
    assert 29 <= sum(x < 10 for x in xs) <= 31


def test_cross_entropy(checkCEConvergence):
    scenario = compileScenic(
        """
        param verifaiSamplerType = "ce"
        from dotmap import DotMap
        ce_params = DotMap()
        ce_params.alpha = 0.99
        ce_params.cont.buckets = 2
        param verifaiSamplerParams = ce_params
        ego = new Object at VerifaiRange(-5, 5) @ 0
        """
    )
    checkCEConvergence(scenario, rangeCheck=(lambda x: -5 <= x <= 5), fast=False)


def test_cross_entropy_weights(checkCEConvergence):
    scenario = compileScenic(
        """
        param verifaiSamplerType = "ce"
        from dotmap import DotMap
        param verifaiSamplerParams = DotMap(alpha=0.99)
        ego = new Object at VerifaiRange(-1, 1, weights=[9, 1]) @ 0
        """
    )

    # Save scenario state which should not be mutated by sampling
    params = scenario.params["verifaiSamplerParams"].copy()
    contCESampler = scenario.externalSampler.sampler.domainSampler.cont_sampler
    prior = contCESampler.dist.copy()

    # Generate samples and check convergence to the correct bucket
    checkCEConvergence(scenario, rangeCheck=(lambda x: -1 <= x <= 1), warmup=1300)

    # Check the scenario state is unchanged
    assert scenario.params["verifaiSamplerParams"] == params
    scenario.resetExternalSampler()
    contCESampler = scenario.externalSampler.sampler.domainSampler.cont_sampler
    assert np.array_equal(contCESampler.dist, prior)

    # Generate new samples without feedback and check the prior distribution is used
    checkCEStationary(scenario)


def test_cross_entropy_options(checkCEConvergence):
    scenario = compileScenic(
        """
        param verifaiSamplerType = "ce"
        from dotmap import DotMap
        param verifaiSamplerParams = DotMap(alpha=0.99)
        ego = new Object at VerifaiOptions({-1: 9, 1: 1}) @ 0
        """
    )
    checkCEConvergence(scenario, warmup=1300)
    scenario.resetExternalSampler()
    checkCEStationary(scenario)


def test_cross_entropy_prior(checkCEConvergence):
    scenario = compileScenic(
        """
        param verifaiSamplerType = "ce"
        from dotmap import DotMap
        param verifaiSamplerParams = DotMap(alpha=0.99)
        ego = new Object at VerifaiParameter.withPrior(Options({-1: 9, 1: 1})) @ 0
        """
    )
    checkCEConvergence(scenario, warmup=1300)
    scenario.resetExternalSampler()
    checkCEStationary(scenario)


def test_cross_entropy_prior_normal(checkCEConvergence):
    scenario = compileScenic(
        """
        param verifaiSamplerType = "ce"
        from dotmap import DotMap
        param verifaiSamplerParams = DotMap(alpha=0.99)
        ego = new Object at VerifaiParameter.withPrior(Normal(-1, 0.8)) @ 0
        """
    )
    checkCEConvergence(scenario, rangeCheck=(lambda x: True), warmup=1300)
    scenario.resetExternalSampler()
    checkCEStationary(scenario)


## Reproducibility and noninterference


def test_reproducibility():
    scenario = compileScenic(
        'param verifaiSamplerType = "ce"\n' "ego = new Object at VerifaiRange(-1, 1) @ 0"
    )
    import numpy

    f = lambda ego: -1 if ego.position.x < 10 else 1

    def sampleSequence(seed):
        random.seed(seed)
        numpy.random.seed(seed)  # TODO improve?
        scenario.resetExternalSampler()
        return [ego.position.x for ego in sampleEgoWithFeedback(scenario, f, 10)]

    seeds = [random.randint(0, 100000) for i in range(10)]
    for seed in seeds:
        base = sampleSequence(seed)
        other = sampleSequence(seed)
        assert base == other


def test_noninterference():
    for i in range(2):
        scenario = compileScenic("ego = new Object at VerifaiRange(0, 1) @ 0")
        for j in range(5):
            scene, iterations = scenario.generate(maxIterations=1)
            assert len(scenario.externalSampler.cachedSample) == 1
