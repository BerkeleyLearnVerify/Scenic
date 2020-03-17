
import warnings

import scipy.stats
import numpy.linalg

from scenic.core.distributions import Range, Normal, TruncatedNormal, DiscreteRange, Options

def similarDistributions(d1, d2, samples=3000, p=0.001):
    s1 = [d1.sample() for i in range(samples)]
    s2 = [d2.sample() for i in range(samples)]
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        try:
            return scipy.stats.epps_singleton_2samp(s1, s2).pvalue > p
        except numpy.linalg.LinAlgError:
            return scipy.stats.ks_2samp(s1, s2).pvalue > p
        

def test_bucketed_range():
    r = Range(-3, 7)
    assert similarDistributions(r, r.bucket())
    assert similarDistributions(r, r.bucket(buckets=100))

def test_bucketed_normal():
    n = Normal(22, 5)
    assert similarDistributions(n, n.bucket())
    assert similarDistributions(n, n.bucket(buckets=15))
    assert similarDistributions(n, n.bucket(buckets=[0, 20, 23, 50]))

def test_bucketed_truncated_normal():
    n = TruncatedNormal(-10, 3, -1, 5)
    assert similarDistributions(n, n.bucket())
    assert similarDistributions(n, n.bucket(buckets=15))
    assert similarDistributions(n, n.bucket(buckets=[-1, -0.5, 4, 5]))

def test_bucketed_discrete_range():
    r = DiscreteRange(-3, 7)
    assert similarDistributions(r, r.bucket())

def test_bucketed_options():
    o = Options({0: 1, 1: 3})
    assert similarDistributions(o, o.bucket())
