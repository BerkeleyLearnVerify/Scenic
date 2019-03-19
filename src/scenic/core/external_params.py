
from dotmap import DotMap

from scenic.core.distributions import Distribution

class ExternalSampler:
	@staticmethod
	def forParameters(params):
		if len(params) > 0:
			return VerifaiSampler(params)   # TODO generalize
		else:
			return None

	def sample(self):
		self.cachedSample = self.nextSample()

	def nextSample(self):
		raise NotImplementedError

	def valueFor(self, param):
		raise NotImplementedError

class VerifaiSampler(ExternalSampler):
	def __init__(self, params):
		import verifai.features
		import verifai.samplers
		self.params = tuple(params)
		for index, param in enumerate(self.params):
			if not isinstance(param, VerifaiParameter):
				raise RuntimeError(f'VerifaiSampler given parameter of wrong type: {param}')
			param.sampler = self
			param.index = index
		space = verifai.features.FeatureSpace({
		    f'param{index}': verifai.features.Feature(param.domain)
		    for index, param in enumerate(self.params)
		})
		halton_params = DotMap()
		halton_params.sample_index = -2
		halton_params.bases_skipped = 0
		self.sampler = verifai.samplers.FeatureSampler.haltonSamplerFor(space, halton_params)

	def nextSample(self):
		return self.sampler.nextSample()

	def valueFor(self, param):
		return self.cachedSample[param.index]

class ExternalParameter(Distribution):
	"""A value determined by external code rather than Scenic's internal sampler."""
	def __init__(self):
		super().__init__()
		self.sampler = None
		import scenic.syntax.veneer as veneer	# TODO improve?
		veneer.registerExternalParameter(self)

	def sampleGiven(self, value):
		assert self.sampler is not None
		return self.sampler.valueFor(self)

class VerifaiParameter(ExternalParameter):
	def __init__(self, domain):
		super().__init__()
		self.domain = domain

class VerifaiRange(VerifaiParameter):
	def __init__(self, low, high):
		import verifai.features
		super().__init__(verifai.features.Box([low, high]))

	def sampleGiven(self, value):
		value = super().sampleGiven(value)
		assert len(value) == 1
		return value[0]
