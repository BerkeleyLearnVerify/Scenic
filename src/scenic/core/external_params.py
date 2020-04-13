
from scenic.core.distributions import Distribution
from scenic.core.utils import InvalidScenarioError

class ExternalSampler:
	def __init__(self, params, globalParams):
		# feedback value passed to external sampler when the last scene was rejected
		self.rejectionFeedback = globalParams.get('externalSamplerRejectionFeedback')

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

	def sample(self, feedback):
		self.cachedSample = self.nextSample(feedback)

	def nextSample(self, feedback):
		raise NotImplementedError

	def valueFor(self, param):
		raise NotImplementedError

class VerifaiSampler(ExternalSampler):
	def __init__(self, params, globalParams):
		super().__init__(params, globalParams)
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
		_, sampler = verifai.server.choose_sampler(space, samplerType,
		                                           sampler_params=samplerParams)
		self.sampler = sampler

		# default rejection feedback is positive so cross-entropy sampler won't update;
		# for other active samplers an appropriate value should be set manually
		if self.rejectionFeedback is None:
			self.rejectionFeedback = 1

	def nextSample(self, feedback):
		return self.sampler.nextSample(feedback)

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

class VerifaiDiscreteRange(VerifaiParameter):
	def __init__(self, low, high):
		import verifai.features
		super().__init__(verifai.features.DiscreteBox([low, high]))

	def sampleGiven(self, value):
		value = super().sampleGiven(value)
		assert len(value) == 1
		return value[0]
