"""Specifiers and associated objects."""

import itertools

from scenic.core.lazy_eval import (DelayedArgument, valueInContext, requiredProperties,
                                   needsLazyEvaluation)
from scenic.core.distributions import toDistribution
from scenic.core.errors import RuntimeParseError

## Specifiers themselves

class Specifier:
	"""Specifier providing a value for a property given dependencies.

	Any optionally-specified properties are evaluated as attributes of the primary value.
	"""
	def __init__(self, prop, value, deps=None, optionals={}):
		self.property = prop
		self.value = value
		if deps is None:
			deps = set()
		deps |= requiredProperties(value)
		if prop in deps:
			raise RuntimeParseError(f'specifier for property {prop} depends on itself')
		self.requiredProperties = deps
		self.optionals = optionals

	def applyTo(self, obj, optionals):
		"""Apply specifier to an object, including the specified optional properties."""
		val = valueInContext(self.value, obj)
		val = toDistribution(val)
		assert not needsLazyEvaluation(val)
		obj._specify(self.property, val)
		for opt in optionals:
			assert opt in self.optionals
			obj._specify(opt, getattr(val, opt))

	def __str__(self):
		return f'<Specifier of {self.property}>'

## Support for property defaults

class PropertyDefault:
	"""A default value, possibly with dependencies."""
	def __init__(self, requiredProperties, attributes, value):
		self.requiredProperties = set(requiredProperties)
		self.value = value

		def enabled(thing, default):
			if thing in attributes:
				attributes.remove(thing)
				return True
			else:
				return default
		self.isAdditive = enabled('additive', False)
		self.isDynamic = enabled('dynamic', False)
		for attr in attributes:
			raise RuntimeParseError(f'unknown property attribute "{attr}"')

	@staticmethod
	def forValue(value):
		if isinstance(value, PropertyDefault):
			return value
		else:
			return PropertyDefault(set(), set(), lambda self: value)

	def resolveFor(self, prop, overriddenDefs):
		"""Create a Specifier for a property from this default and any superclass defaults."""
		if self.isAdditive:
			allReqs = self.requiredProperties
			for other in overriddenDefs:
				allReqs |= other.requiredProperties
			def concatenator(context):
				allVals = [self.value(context)]
				for other in overriddenDefs:
					allVals.append(other.value(context))
				return tuple(allVals)
			val = DelayedArgument(allReqs, concatenator, _internal=True)
		else:
			val = DelayedArgument(self.requiredProperties, self.value, _internal=True)
		return Specifier(prop, val)
