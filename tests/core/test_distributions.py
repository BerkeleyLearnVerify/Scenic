
import math
import typing
import warnings

import pytest
import scipy.stats
import numpy.linalg

from scenic.core.distributions import (Range, Normal, TruncatedNormal, DiscreteRange, Options,
                                       distributionFunction)

# Bucketing

@pytest.fixture
def similarDistributions(pytestconfig):
    samples = 3000 if pytestconfig.getoption('--fast') else 100000

    def checker(d1, d2, p=1e-4):
        s1 = [d1.sample() for i in range(samples)]
        s2 = [d2.sample() for i in range(samples)]
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            try:
                assert scipy.stats.epps_singleton_2samp(s1, s2).pvalue > p, p
            except numpy.linalg.LinAlgError:
                assert scipy.stats.ks_2samp(s1, s2).pvalue > p, p

    return checker

def test_bucketed_range(similarDistributions):
    r = Range(-3, 7)
    similarDistributions(r, r.bucket())
    similarDistributions(r, r.bucket(buckets=100))

def test_bucketed_normal(similarDistributions):
    n = Normal(22, 5)
    similarDistributions(n, n.bucket())
    similarDistributions(n, n.bucket(buckets=15))
    similarDistributions(n, n.bucket(buckets=[0, 20, 23, 50]))

def test_bucketed_truncated_normal(similarDistributions):
    n = TruncatedNormal(-10, 3, -1, 5)
    similarDistributions(n, n.bucket())
    similarDistributions(n, n.bucket(buckets=15))
    similarDistributions(n, n.bucket(buckets=[-1, -0.5, 4, 5]))

def test_bucketed_discrete_range(similarDistributions):
    r = DiscreteRange(-3, 7)
    similarDistributions(r, r.bucket())

def test_bucketed_options(similarDistributions):
    o = Options({0: 1, 1: 3})
    similarDistributions(o, o.bucket())

# Decorators for supporting random arguments

def test_function_decorator():
    @distributionFunction
    def myfunc(x, y):
        if x > 0:
            return (x, y)
        else:
            return (x, -y)
    res = myfunc(Range(-1, 1), 42)
    for i in range(30):
        z, w = res.sample()
        assert w == 42 if z > 0 else w == -42

def test_function_decorator_builtin():
    myprint = distributionFunction(print)
    res = myprint(Options([7, 9]))
    res()
    myhypot = distributionFunction(math.hypot)
    res = myhypot(4, Options([-3, 3]))
    for i in range(30):
        assert res.sample() == pytest.approx(5)

def test_function_decorator_annotations():
    @distributionFunction
    def myfunc(x: float) -> float:
        return 2 * x
    assert typing.get_type_hints(myfunc) == {'x': float, 'return': float}
