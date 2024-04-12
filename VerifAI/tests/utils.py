
import os.path

from verifai.samplers import FeatureSampler

def sampleWithFeedback(sampler, num_samples, f):
    feedback = None
    samples = []
    for i in range(num_samples):
        sample = sampler.nextSample(feedback)
        feedback = f(sample)
        print(f'Sample #{i}:')
        print(sample)
        samples.append(sample)
    return samples

def checkSaveRestore(sampler, tmpdir, iterations=1):
    path = os.path.join(tmpdir, 'blah.dat')
    feedback = None
    for i in range(iterations):
        sampler.saveToFile(path)
        sample1 = sampler.nextSample(feedback)
        sample2 = sampler.nextSample(-1)
        sampler = FeatureSampler.restoreFromFile(path)
        sample1b = sampler.nextSample(feedback)
        sample2b = sampler.nextSample(-1)
        assert sample1 != sample2
        assert sample1 == sample1b
        assert sample2 == sample2b
        sampler.saveToFile(path)
        sample3 = sampler.nextSample(1)
        sampler = FeatureSampler.restoreFromFile(path)
        sample3b = sampler.nextSample(1)
        assert sample3 not in (sample1, sample2)
        assert sample3 == sample3b
        feedback = 1
