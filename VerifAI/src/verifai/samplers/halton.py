"""Halton sampler"""

from verifai.samplers.domain_sampler import BoxSampler
import itertools
from math import floor

def generate_primes():
    D = {}
    yield 2
    for q in itertools.islice(itertools.count(3), 0, None, 2):
        p = D.pop(q, None)
        if p is None:
            D[q * q] = q
            yield q
        else:
            x = q + 2 * p
            while x in D:
                x += 2 * p
            D[x] = p

def halton_sequence(index, base):
    f,r = 1, 0
    while index > 0:
        f = f/base
        r += f*(index%base)
        index = floor(index/base)
    return r

class HaltonSampler(BoxSampler):
    """Samples along a quasi-random Halton sequence"""

    def __init__(self, domain, halton_params):
        super().__init__(domain)
        self.sample_index = halton_params.sample_index
        primes = generate_primes()
        for _ in range(halton_params.bases_skipped):
            next(primes)
        self.prime_bases = [next(primes) for _ in range(self.dimension)]

    def getVector(self):
        self.sample_index += 1
        return tuple(halton_sequence(index=self.sample_index, base=p)
                     for p in self.prime_bases), None
