"""Support for lazy evaluation of expressions and specifiers.

Lazy evaluation is necessary for expressions like :samp:`30 deg relative to roadDirection`
where :samp:`roadDirection` is a vector field and so defines a different heading at
different positions. Scenic defers evaluation of such expressions until they are used in
the definition of an object, when the required context (here, a position) is available.
This is implemented by representing lazy values as special objects which capture all
operations applied to them (in a similar way to `Distribution` objects). The main class
of such objects is `DelayedArgument`: in the above example, the :samp:`relative to`
operator returns such an object. However, since lazy values can appear as arguments to
distributions, `Distribution` objects can also require lazy evaluation (prior to
sampling); therefore both of these classes derive from a common abstract class
`LazilyEvaluable`.
"""

import itertools
import types

from scenic.core.utils import DefaultIdentityDict

class LazilyEvaluable:
	"""Values which may require evaluation in the context of an object being constructed.

	If a LazilyEvaluable specifies any properties it depends on, then it cannot be evaluated to a
	normal value except during the construction of an object which already has values for those
	properties.

	Args:
		requiredProps: sequence of strings naming all properties which this value can
			depend on (formally, which must exist in the object passed as the context to
			`evaluateIn`).

	Attributes:
		_requiredProperties: set of strings as above.
	"""
	def __init__(self, requiredProps):
		self._dependencies = ()		# TODO improve?
		self._requiredProperties = set(requiredProps)

	def evaluateIn(self, context):
		"""Evaluate this value in the context of an object being constructed.

		The object must define all of the properties on which this value depends.
		"""
		cache = context._evaluated		# cache of lazy values already evaluated in this context
		if self in cache:
			return cache[self]	# avoid making a new evaluated copy of this value
		assert all(hasattr(context, prop) for prop in self._requiredProperties)
		value = self.evaluateInner(context)
		assert not needsLazyEvaluation(value)	# value should not require further evaluation
		cache[self] = value
		return value

	def evaluateInner(self, context):
		"""Actually evaluate in the given context, which provides all required properties.

		Overridden by subclasses.
		"""
		return self

	@staticmethod
	def makeContext(**props):
		"""Make a context with the given properties for testing purposes."""
		context = types.SimpleNamespace(**props)
		context._evaluated = DefaultIdentityDict()
		return context

class DelayedArgument(LazilyEvaluable):
	"""DelayedArgument(requiredProps, value, _internal=False)

	Specifier arguments requiring other properties to be evaluated first.

	The value of a DelayedArgument is given by a function mapping the context (object under
	construction) to a value.

	.. note::

		When called from a dynamic behavior, constructors for delayed arguments return
		*actual evaluations*, not `DelayedArgument` objects. The agent running the
		behavior is used as the evaluation context.

	Args:
		requiredProps: see `LazilyEvaluable`.
		value: function taking a single argument (the context) and returning the
			corresponding evaluation of this object.
		_internal (bool): set to `True` for internal uses that need to suppress the
			exceptional handling of calls from dynamic behaviors above.
	"""
	def __new__(cls, *args, _internal=False, **kwargs):
		darg = super().__new__(cls)
		if _internal:
			return darg
		# at runtime, evaluate immediately in the context of the current agent
		import scenic.syntax.veneer as veneer
		if veneer.simulationInProgress() and veneer.currentBehavior:
			agent = veneer.currentBehavior._agent
			assert agent
			darg.__init__(*args, **kwargs)
			agent._evaluated = DefaultIdentityDict()
			value = darg.evaluateIn(agent)
			del agent._evaluated
			return value
		else:
			return darg

	def __init__(self, requiredProps, value, _internal=False):
		self.value = value
		super().__init__(requiredProps)

	def evaluateInner(self, context):
		return self.value(context)

	def __getattr__(self, name):
		return DelayedArgument(self._requiredProperties,
			lambda context: getattr(self.evaluateIn(context), name),
			_internal=True)

	def __call__(self, *args, **kwargs):
		subprops = (requiredProperties(arg) for arg in itertools.chain(args, kwargs.values()))
		props = self._requiredProperties.union(*subprops)
		def value(context):
			subvalues = (valueInContext(arg, context) for arg in args)
			kwsvs = { name: valueInContext(arg, context) for name, arg in kwargs.items() }
			return self.evaluateIn(context)(*subvalues, **kwsvs)
		return DelayedArgument(props, value, _internal=True)

# Operators which can be applied to DelayedArguments
allowedOperators = [
	'__neg__',
	'__pos__',
	'__abs__',
	'__lt__', '__le__',
	'__eq__', '__ne__',
	'__gt__', '__ge__',
	'__add__', '__radd__',
	'__sub__', '__rsub__',
	'__mul__', '__rmul__',
	'__truediv__', '__rtruediv__',
	'__floordiv__', '__rfloordiv__',
	'__mod__', '__rmod__',
	'__divmod__', '__rdivmod__',
	'__pow__', '__rpow__',
	'__round__',
	'__len__',
	'__getitem__'
]
def makeDelayedOperatorHandler(op):
	def handler(self, *args):
		props = self._requiredProperties.union(*(requiredProperties(arg) for arg in args))
		def value(context):
			subvalues = (valueInContext(arg, context) for arg in args)
			return getattr(self.evaluateIn(context), op)(*subvalues)
		return DelayedArgument(props, value, _internal=True)
	return handler
for op in allowedOperators:
	setattr(DelayedArgument, op, makeDelayedOperatorHandler(op))

def makeDelayedFunctionCall(func, args, kwargs):
	"""Utility function for creating a lazily-evaluated function call."""
	props = set().union(*(requiredProperties(arg)
	                      for arg in itertools.chain(args, kwargs.values())))
	def value(context):
		subvalues = (valueInContext(arg, context) for arg in args)
		kwsubvals = { name: valueInContext(arg, context) for name, arg in kwargs.items() }
		return func(*subvalues, **kwsubvals)
	return DelayedArgument(props, value, _internal=True)

def valueInContext(value, context):
	"""Evaluate something in the context of an object being constructed."""
	if needsLazyEvaluation(value):
		return value.evaluateIn(context)
	else:
		return value

def requiredProperties(thing):
	"""Set of properties needed to evaluate the given value, if any."""
	if isinstance(thing, LazilyEvaluable):
		return thing._requiredProperties
	else:
		return set()

def needsLazyEvaluation(thing):
	"""Whether the given value requires lazy evaluation."""
	return isinstance(thing, DelayedArgument) or requiredProperties(thing)
