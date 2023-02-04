
"""Support for lazy evaluation of expressions and specifiers."""

import itertools

class LazilyEvaluable:
	"""Values which may require evaluation in the context of an object being constructed.

	If a LazilyEvaluable specifies any properties it depends on, then it cannot be evaluated to a
	normal value except during the construction of an object which already has values for those
	properties.
	"""
	def __init__(self, requiredProps):
		self._dependencies = ()		# TODO improve?
		self._requiredProperties = set(requiredProps)

	def evaluateIn(self, context):
		"""Evaluate this value in the context of an object being constructed.

		The object must define all of the properties on which this value depends.
		"""
		assert all(hasattr(context, prop) for prop in self._requiredProperties)
		value = self.evaluateInner(context)
		assert not needsLazyEvaluation(value)	# value should not require further evaluation
		return value

	def evaluateInner(self, context):
		"""Actually evaluate in the given context, which provides all required properties."""
		return self

class DelayedArgument(LazilyEvaluable):
	"""Specifier arguments requiring other properties to be evaluated first.

	The value of a DelayedArgument is given by a function mapping the context (object under
	construction) to a value.
	"""
	def __new__(cls, *args, _internal=False, **kwargs):
		darg = super().__new__(cls)
		if _internal:
			return darg
		# at runtime, evaluate immediately in the context of the current agent
		import scenic.syntax.veneer as veneer
		if veneer.simulationInProgress() and veneer.currentBehavior:
			behavior = veneer.currentBehavior
			assert behavior.agent
			darg.__init__(*args, **kwargs)
			return darg.evaluateIn(behavior.agent)
		else:
			return darg

	def __init__(self, requiredProps, value, _internal=False):
		self.value = value
		super().__init__(requiredProps)

	def evaluateInner(self, context):
		return self.value(context)

	def __getattr__(self, name):
		return DelayedArgument(self._requiredProperties,
			lambda context: getattr(self.evaluateIn(context), name))

	def __call__(self, *args, **kwargs):
		dargs = [toDelayedArgument(arg) for arg in args]
		kwdargs = { name: toDelayedArgument(arg) for name, arg in kwargs.items() }
		subprops = (darg._requiredProperties for darg in itertools.chain(dargs, kwdargs.values()))
		props = self._requiredProperties.union(*subprops)
		def value(context):
			subvalues = (darg.evaluateIn(context) for darg in dargs)
			kwsvs = { name: darg.evaluateIn(context) for name, darg in kwdargs.items() }
			return self.evaluateIn(context)(*subvalues, **kwsvs)
		return DelayedArgument(props, value)

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
		dargs = [toDelayedArgument(arg) for arg in args]
		props = self._requiredProperties.union(*(darg._requiredProperties for darg in dargs))
		def value(context):
			subvalues = (darg.evaluateIn(context) for darg in dargs)
			return getattr(self.evaluateIn(context), op)(*subvalues)
		return DelayedArgument(props, value)
	return handler
for op in allowedOperators:
	setattr(DelayedArgument, op, makeDelayedOperatorHandler(op))

def makeDelayedFunctionCall(func, args, kwargs):
	"""Utility function for creating a lazily-evaluated function call."""
	dargs = [toDelayedArgument(arg) for arg in args]
	kwdargs = { name: toDelayedArgument(arg) for name, arg in kwargs.items() }
	props = set().union(*(darg._requiredProperties
	                      for darg in itertools.chain(dargs, kwdargs.values())))
	def value(context):
		subvalues = (darg.evaluateIn(context) for darg in dargs)
		kwsubvals = { name: darg.evaluateIn(context) for name, darg in kwdargs.items() }
		return func(*subvalues, **kwsubvals)
	return DelayedArgument(props, value)

def valueInContext(value, context):
	"""Evaluate something in the context of an object being constructed."""
	try:
		return value.evaluateIn(context)
	except AttributeError:
		return value

def toDelayedArgument(thing, internal=False):
	if isinstance(thing, DelayedArgument):
		return thing
	return DelayedArgument(set(), lambda context: thing, _internal=internal)

def requiredProperties(thing):
	if hasattr(thing, '_requiredProperties'):
		return thing._requiredProperties
	return set()

def needsLazyEvaluation(thing):
	return isinstance(thing, DelayedArgument) or requiredProperties(thing)
