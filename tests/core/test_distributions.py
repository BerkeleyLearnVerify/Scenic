
import warnings

import scipy.stats
import numpy.linalg

from scenic.core.distributions import Range, Normal, TruncatedNormal, DiscreteRange, Options

def similarDistributions(d1, d2, samples=3000, p=0.002):
    s1 = [d1.sample() for i in range(samples)]
    s2 = [d2.sample() for i in range(samples)]
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        try:
            assert scipy.stats.epps_singleton_2samp(s1, s2).pvalue > p, p
        except numpy.linalg.LinAlgError:
            assert scipy.stats.ks_2samp(s1, s2).pvalue > p, p


def test_bucketed_range():
    r = Range(-3, 7)
    similarDistributions(r, r.bucket())
    similarDistributions(r, r.bucket(buckets=100))

def test_bucketed_normal():
    n = Normal(22, 5)
    similarDistributions(n, n.bucket())
    similarDistributions(n, n.bucket(buckets=15))
    similarDistributions(n, n.bucket(buckets=[0, 20, 23, 50]))

def test_bucketed_truncated_normal():
    n = TruncatedNormal(-10, 3, -1, 5)
    similarDistributions(n, n.bucket())
    similarDistributions(n, n.bucket(buckets=15))
    similarDistributions(n, n.bucket(buckets=[-1, -0.5, 4, 5]))

def test_bucketed_discrete_range():
    r = DiscreteRange(-3, 7)
    similarDistributions(r, r.bucket())

def test_bucketed_options():
    o = Options({0: 1, 1: 3})
    similarDistributions(o, o.bucket())
