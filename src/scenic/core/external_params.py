
from dotmap import DotMap

from scenic.core.distributions import Distribution
from scenic.core.utils import InvalidScenarioError

class ExternalSampler:
	@staticmethod
	def forParameters(params, globalParams):
		if len(params) > 0:
			externalSampler = globalParams.get('externalSampler', VerifaiSampler)
			if not issubclass(externalSampler, ExternalSampler):
				raise InvalidScenarioError(f'externalSampler type {externalSampler}'
				                           ' not subclass of ExternalSampler')
			return externalSampler(params, globalParams)
		else:
			return None

	def sample(self):
		self.cachedSample = self.nextSample()

	def nextSample(self):
		raise NotImplementedError

	def valueFor(self, param):
		raise NotImplementedError

class VerifaiSampler(ExternalSampler):
	def __init__(self, params, globalParams):
		import verifai.features
		import verifai.server

		# construct FeatureSpace
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

		# set up VerifAI sampler
		samplerType = globalParams.get('verifaiSamplerType', 'halton')
		samplerParams = globalParams.get('verifaiSamplerParams', None)
		samplerFunc = globalParams.get('verifaiSamplerFunc', None)
		_, sampler = verifai.server.choose_sampler(space, samplerType,
		                                           sampler_params=samplerParams,
		                                           sampler_func=samplerFunc)
		self.sampler = sampler

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
