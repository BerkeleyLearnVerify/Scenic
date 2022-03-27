
"""Python implementations of Scenic language constructs.

This module is automatically imported by all Scenic programs. In addition to
defining the built-in functions, operators, specifiers, etc., it also stores
global state such as the list of all created Scenic objects.
"""

__all__ = (
	# Primitive statements and functions
	'ego', 'require', 'resample', 'param', 'globalParameters', 'mutate', 'verbosePrint',
	'localPath', 'model', 'simulator', 'simulation', 'require_always', 'require_eventually',
	'terminate_when', 'terminate_simulation_when', 'terminate_after', 'in_initial_scenario',
	'override',
	'record', 'record_initial', 'record_final',
	'sin', 'cos', 'hypot', 'max', 'min',
	'filter', 'str',
	# Prefix operators
	'Visible', 'NotVisible',
	'Front', 'Back', 'Left', 'Right',
	'FrontLeft', 'FrontRight', 'BackLeft', 'BackRight',
	'RelativeHeading', 'ApparentHeading', 'RelativePosition',
	'DistanceFrom', 'DistancePast', 'AngleTo', 'AngleFrom', 'Follow',
	# Infix operators
	'FieldAt', 'RelativeTo', 'OffsetAlong', 'CanSee',
	# Primitive types
	'Vector', 'VectorField', 'PolygonalVectorField',
	'Region', 'PointSetRegion', 'RectangularRegion', 'CircularRegion', 'SectorRegion',
	'PolygonalRegion', 'PolylineRegion',
	'Workspace', 'Mutator',
	'Range', 'DiscreteRange', 'Options', 'Uniform', 'Discrete', 'Normal',
	'TruncatedNormal',
	'VerifaiParameter', 'VerifaiRange', 'VerifaiDiscreteRange', 'VerifaiOptions',
	# Constructible types
	'Point', 'OrientedPoint', 'Object',
	# Specifiers
	'With',
	'At', 'In', 'Beyond', 'VisibleFrom', 'VisibleSpec', 'OffsetBy', 'OffsetAlongSpec',
	'Facing', 'FacingToward', 'ApparentlyFacing',
	'LeftSpec', 'RightSpec', 'Ahead', 'Behind',
	'Following',
	# Constants
	'everywhere', 'nowhere',
	# Exceptions
	'GuardViolation', 'PreconditionViolation', 'InvariantViolation',
	# Internal APIs 	# TODO remove?
	'PropertyDefault', 'Behavior', 'Monitor', 'makeTerminationAction',
	'BlockConclusion', 'runTryInterrupt', 'wrapStarredValue', 'callWithStarArgs',
	'Modifier', 'DynamicScenario'
)

# various Python types and functions used in the language but defined elsewhere
from scenic.core.geometry import sin, cos, hypot, max, min
from scenic.core.vectors import Vector, VectorField, PolygonalVectorField
from scenic.core.regions import (Region, PointSetRegion, RectangularRegion,
	CircularRegion, SectorRegion, PolygonalRegion, PolylineRegion,
	everywhere, nowhere)
from scenic.core.workspaces import Workspace
from scenic.core.distributions import (Range, DiscreteRange, Options, Uniform, Normal,
	TruncatedNormal)
Discrete = Options
from scenic.core.external_params import (VerifaiParameter, VerifaiRange, VerifaiDiscreteRange,
										 VerifaiOptions)
from scenic.core.object_types import Mutator, Point, OrientedPoint, Object
from scenic.core.specifiers import PropertyDefault	# TODO remove
from scenic.core.dynamics import (Behavior, Monitor, DynamicScenario, BlockConclusion,
                                  GuardViolation, PreconditionViolation, InvariantViolation,
                                  makeTerminationAction, runTryInterrupt)

# everything that should not be directly accessible from the language is imported here:
import builtins
import collections.abc
from contextlib import contextmanager
import importlib
import sys
import random
import os.path
import traceback
import typing
from scenic.core.distributions import (RejectionException, Distribution,
									   TupleDistribution, StarredDistribution, toDistribution,
									   needsSampling, canUnpackDistributions, distributionFunction)
from scenic.core.type_support import (isA, toType, toTypes, toScalar, toHeading, toVector,
									  evaluateRequiringEqualTypes, underlyingType,
									  canCoerce, coerce)
from scenic.core.geometry import normalizeAngle, apparentHeadingAtPoint
from scenic.core.object_types import _Constructible
from scenic.core.specifiers import Specifier
from scenic.core.lazy_eval import DelayedArgument, needsLazyEvaluation
from scenic.core.errors import RuntimeParseError, InvalidScenarioError
from scenic.core.vectors import OrientedVector
from scenic.core.external_params import ExternalParameter
import scenic.core.requirements as requirements
from scenic.core.simulators import RejectSimulationException

### Internals

activity = 0
currentScenario = None
scenarioStack = []
scenarios = []
evaluatingRequirement = False
_globalParameters = {}
lockedParameters = set()
lockedModel = None
loadingModel = False
currentSimulation = None
inInitialScenario = True
runningScenarios = set()
currentBehavior = None
simulatorFactory = None
evaluatingGuard = False

## APIs used internally by the rest of Scenic

# Scenic compilation

def isActive():
	"""Are we in the middle of compiling a Scenic module?

	The 'activity' global can be >1 when Scenic modules in turn import other
	Scenic modules.
	"""
	return activity > 0

def activate(paramOverrides={}, modelOverride=None, filename=None, namespace=None):
	"""Activate the veneer when beginning to compile a Scenic module."""
	global activity, _globalParameters, lockedParameters, lockedModel, currentScenario
	if paramOverrides or modelOverride:
		assert activity == 0
		_globalParameters.update(paramOverrides)
		lockedParameters = set(paramOverrides)
		lockedModel = modelOverride

	activity += 1
	assert not evaluatingRequirement
	assert not evaluatingGuard
	assert currentSimulation is None
	# placeholder scenario for top-level code
	newScenario = DynamicScenario._dummy(filename, namespace)
	scenarioStack.append(newScenario)
	currentScenario = newScenario

def deactivate():
	"""Deactivate the veneer after compiling a Scenic module."""
	global activity, _globalParameters, lockedParameters, lockedModel
	global currentScenario, scenarios, scenarioStack, simulatorFactory
	activity -= 1
	assert activity >= 0
	assert not evaluatingRequirement
	assert not evaluatingGuard
	assert currentSimulation is None
	scenarioStack.pop()
	assert len(scenarioStack) == activity
	scenarios = []

	if activity == 0:
		lockedParameters = set()
		lockedModel = None
		currentScenario = None
		simulatorFactory = None
		_globalParameters = {}
	else:
		currentScenario = scenarioStack[-1]

# Object creation

def registerObject(obj):
	"""Add a Scenic object to the global list of created objects.

	This is called by the Object constructor.
	"""
	if evaluatingRequirement:
		raise RuntimeParseError('tried to create an object inside a requirement')
	elif currentBehavior is not None:
		raise RuntimeParseError('tried to create an object inside a behavior')
	elif activity > 0 or currentScenario:
		assert not evaluatingRequirement
		assert isinstance(obj, _Constructible)
		currentScenario._registerObject(obj)
		if currentSimulation:
			currentSimulation.createObject(obj)

# External parameter creation

def registerExternalParameter(value):
	"""Register a parameter whose value is given by an external sampler."""
	if activity > 0:
		assert isinstance(value, ExternalParameter)
		currentScenario._externalParameters.append(value)

# Function call support

def wrapStarredValue(value, lineno):
	if isinstance(value, TupleDistribution) or not needsSampling(value):
		return value
	elif isinstance(value, Distribution):
		return [StarredDistribution(value, lineno)]
	else:
		raise RuntimeParseError(f'iterable unpacking cannot be applied to {value}')

def callWithStarArgs(_func_to_call, *args, **kwargs):
	if not canUnpackDistributions(_func_to_call):
		# wrap function to delay evaluation until starred distributions are sampled
		_func_to_call = distributionFunction(_func_to_call)
	return _func_to_call(*args, **kwargs)

# Simulations

def instantiateSimulator(factory, params):
	global _globalParameters
	assert not _globalParameters		# TODO improve hack?
	_globalParameters = dict(params)
	try:
		return factory()
	finally:
		_globalParameters = {}

def beginSimulation(sim):
	global currentSimulation, currentScenario, inInitialScenario, runningScenarios
	global _globalParameters
	if isActive():
		raise RuntimeError('tried to start simulation during Scenic compilation!')
	assert currentSimulation is None
	assert currentScenario is None
	assert not scenarioStack
	currentSimulation = sim
	currentScenario = sim.scene.dynamicScenario
	runningScenarios = {currentScenario}
	inInitialScenario = currentScenario._setup is None
	currentScenario._bindTo(sim.scene)
	_globalParameters = dict(sim.scene.params)

	# rebind globals that could be referenced by behaviors to their sampled values
	for modName, (namespace, sampledNS, originalNS) in sim.scene.behaviorNamespaces.items():
		namespace.clear()
		namespace.update(sampledNS)

def endSimulation(sim):
	global currentSimulation, currentScenario, currentBehavior, runningScenarios
	global _globalParameters
	currentSimulation = None
	currentScenario = None
	runningScenarios = set()
	currentBehavior = None
	_globalParameters = {}

	for modName, (namespace, sampledNS, originalNS) in sim.scene.behaviorNamespaces.items():
		namespace.clear()
		namespace.update(originalNS)

def simulationInProgress():
	return currentSimulation is not None

# Requirements

@contextmanager
def executeInRequirement(scenario, boundEgo):
	global evaluatingRequirement, currentScenario
	assert not evaluatingRequirement
	evaluatingRequirement = True
	if currentScenario is None:
		currentScenario = scenario
		clearScenario = True
	else:
		assert currentScenario is scenario
		clearScenario = False
	oldEgo = currentScenario._ego
	if boundEgo:
		currentScenario._ego = boundEgo
	try:
		yield
	finally:
		evaluatingRequirement = False
		currentScenario._ego = oldEgo
		if clearScenario:
			currentScenario = None

# Dynamic scenarios

def registerDynamicScenarioClass(cls):
	scenarios.append(cls)

@contextmanager
def executeInScenario(scenario, inheritEgo=False):
	global currentScenario
	oldScenario = currentScenario
	if inheritEgo and oldScenario is not None:
		scenario._ego = oldScenario._ego 	# inherit ego from parent
	currentScenario = scenario
	try:
		yield
	except AttributeError as e:
		# Convert confusing AttributeErrors from trying to access nonexistent scenario
		# variables into NameErrors, which is what the user would expect. The information
		# needed to do this was made available in Python 3.10, but unfortunately could be
		# wrong until 3.10.3: see bpo-46940.
		if sys.version_info >= (3, 10, 3) and isinstance(e.obj, DynamicScenario):
			newExc = NameError(f"name '{e.name}' is not defined", name=e.name)
			raise newExc.with_traceback(e.__traceback__)
		else:
			raise
	finally:
		currentScenario = oldScenario

def prepareScenario(scenario):
	if currentSimulation:
		verbosePrint(f'Starting scenario {scenario}', level=3)

def finishScenarioSetup(scenario):
	global inInitialScenario
	inInitialScenario = False

def startScenario(scenario):
	runningScenarios.add(scenario)

def endScenario(scenario, reason, quiet=False):
	runningScenarios.remove(scenario)
	if not quiet:
		verbosePrint(f'Stopping scenario {scenario} because: {reason}', level=3)

# Dynamic behaviors

@contextmanager
def executeInBehavior(behavior):
	global currentBehavior
	oldBehavior = currentBehavior
	currentBehavior = behavior
	try:
		yield
	except AttributeError as e:
		# See comment for corresponding code in executeInScenario
		if sys.version_info >= (3, 10, 3) and isinstance(e.obj, Behavior):
			newExc = NameError(f"name '{e.name}' is not defined", name=e.name)
			raise newExc.with_traceback(e.__traceback__)
		else:
			raise
	finally:
		currentBehavior = oldBehavior

@contextmanager
def executeInGuard():
	global evaluatingGuard
	assert not evaluatingGuard
	evaluatingGuard = True
	try:
		yield
	finally:
		evaluatingGuard = False

### Parsing support

class Modifier(typing.NamedTuple):
	name: str
	value: typing.Any
	terminator: typing.Optional[str] = None

### Primitive statements and functions

def ego(obj=None):
	"""Function implementing loads and stores to the 'ego' pseudo-variable.

	The translator calls this with no arguments for loads, and with the source
	value for stores.
	"""
	egoObject = currentScenario._ego
	if obj is None:
		if egoObject is None:
			raise RuntimeParseError('referred to ego object not yet assigned')
	elif not isinstance(obj, Object):
		raise RuntimeParseError('tried to make non-object the ego object')
	else:
		currentScenario._ego = obj
		for scenario in runningScenarios:
			if scenario._ego is None:
				scenario._ego = obj
	return egoObject

def require(reqID, req, line, name, prob=1):
	"""Function implementing the require statement."""
	if not name:
		name = f'requirement on line {line}'
	if evaluatingRequirement:
		raise RuntimeParseError('tried to create a requirement inside a requirement')
	if currentSimulation is not None:	# requirement being evaluated at runtime
		if prob >= 1 or random.random() <= prob:
			result = req()
			assert not needsSampling(result)
			if needsLazyEvaluation(result):
				raise RuntimeParseError(f'requirement on line {line} uses value'
										' undefined outside of object definition')
			if not result:
				raise RejectSimulationException(name)
	else:	# requirement being defined at compile time
		currentScenario._addRequirement(requirements.RequirementType.require,
                                        reqID, req, line, name, prob)

def record(reqID, value, line, name):
	if not name:
		name = f'record{line}'
	makeRequirement(requirements.RequirementType.record, reqID, value, line, name)

def record_initial(reqID, value, line, name):
	if not name:
		name = f'record{line}'
	makeRequirement(requirements.RequirementType.recordInitial, reqID, value, line, name)

def record_final(reqID, value, line, name):
	if not name:
		name = f'record{line}'
	makeRequirement(requirements.RequirementType.recordFinal, reqID, value, line, name)

def require_always(reqID, req, line, name):
	"""Function implementing the 'require always' statement."""
	if not name:
		name = f'requirement on line {line}'
	makeRequirement(requirements.RequirementType.requireAlways, reqID, req, line, name)

def require_eventually(reqID, req, line, name):
	"""Function implementing the 'require eventually' statement."""
	if not name:
		name = f'requirement on line {line}'
	makeRequirement(requirements.RequirementType.requireEventually, reqID, req, line, name)


def terminate_when(reqID, req, line, name):
	"""Function implementing the 'terminate when' statement."""
	if not name:
		name = f'termination condition on line {line}'
	makeRequirement(requirements.RequirementType.terminateWhen, reqID, req, line, name)

def terminate_simulation_when(reqID, req, line, name):
	"""Function implementing the 'terminate simulation when' statement."""
	if not name:
		name = f'termination condition on line {line}'
	makeRequirement(requirements.RequirementType.terminateSimulationWhen,
                    reqID, req, line, name)

def makeRequirement(ty, reqID, req, line, name):
	if evaluatingRequirement:
		raise RuntimeParseError(f'tried to use "{ty.value}" inside a requirement')
	elif currentBehavior is not None:
		raise RuntimeParseError(f'"{ty.value}" inside a behavior on line {line}')
	elif currentSimulation is not None:
		currentScenario._addDynamicRequirement(ty, req, line, name)
	else:	# requirement being defined at compile time
		currentScenario._addRequirement(ty, reqID, req, line, name, 1)

def terminate_after(timeLimit, terminator=None):
	if not isinstance(timeLimit, (float, int)):
		raise RuntimeParseError('"terminate after N" with N not a number')
	assert terminator in (None, 'seconds', 'steps')
	inSeconds = (terminator != 'steps')
	currentScenario._setTimeLimit(timeLimit, inSeconds=inSeconds)

def resample(dist):
	"""The built-in resample function."""
	return dist.clone() if isinstance(dist, Distribution) else dist

def verbosePrint(msg, file=sys.stdout, level=1):
	"""Built-in function printing a message when the verbosity is >0.

	(Or when the verbosity exceeds the specified level.)
	"""
	import scenic.syntax.translator as translator
	if translator.verbosity >= level:
		if currentSimulation:
			indent = '      ' if translator.verbosity >= 3 else '  '
		else:
			indent = '  ' * activity if translator.verbosity >= 2 else '  '
		print(indent + msg, file=file)

def localPath(relpath):
	filename = traceback.extract_stack(limit=2)[0].filename
	base = os.path.dirname(filename)
	return os.path.join(base, relpath)

def simulation():
	if isActive():
		raise RuntimeParseError('used simulation() outside a behavior')
	assert currentSimulation is not None
	return currentSimulation

def simulator(sim):
	global simulatorFactory
	simulatorFactory = sim

def in_initial_scenario():
	return inInitialScenario

def override(*args):
	if len(args) < 1:
		raise RuntimeParseError('"override" missing an object')
	elif len(args) < 2:
		raise RuntimeParseError('"override" missing a list of specifiers')
	obj = args[0]
	if not isinstance(obj, Object):
		raise RuntimeParseError(f'"override" passed non-Object {obj}')
	specs = args[1:]
	for spec in specs:
		if not isinstance(spec, Specifier):
			raise RuntimeParseError(f'"override" passed non-specifier {spec}')

	currentScenario._override(obj, specs)

def model(namespace, modelName):
	global loadingModel
	if loadingModel:
		raise RuntimeParseError('Scenic world model itself uses the "model" statement')
	if lockedModel is not None:
		modelName = lockedModel
	try:
		loadingModel = True
		module = importlib.import_module(modelName)
	except ModuleNotFoundError as e:
		if e.name == modelName:
			raise InvalidScenarioError(f'could not import world model {modelName}') from None
		else:
			raise
	finally:
		loadingModel = False
	names = module.__dict__.get('__all__', None)
	if names is not None:
		for name in names:
			namespace[name] = getattr(module, name)
	else:
		for name, value in module.__dict__.items():
			if not name.startswith('_'):
				namespace[name] = value

def param(*quotedParams, **params):
	"""Function implementing the param statement."""
	global loadingModel
	if evaluatingRequirement:
		raise RuntimeParseError('tried to create a global parameter inside a requirement')
	elif currentSimulation is not None:
		raise RuntimeParseError('tried to create a global parameter during a simulation')
	for name, value in params.items():
		if name not in lockedParameters and (not loadingModel or name not in _globalParameters):
			_globalParameters[name] = toDistribution(value)
	assert len(quotedParams) % 2 == 0, quotedParams
	it = iter(quotedParams)
	for name, value in zip(it, it):
		if name not in lockedParameters:
			_globalParameters[name] = toDistribution(value)

class ParameterTableProxy(collections.abc.Mapping):
	def __init__(self, map):
		self._internal_map = map

	def __getitem__(self, name):
		return self._internal_map[name]

	def __iter__(self):
		return iter(self._internal_map)

	def __len__(self):
		return len(self._internal_map)

	def __getattr__(self, name):
		return self.__getitem__(name)	# allow namedtuple-like access

	def _clone_table(self):
		return ParameterTableProxy(self._internal_map.copy())

def globalParameters():
	return ParameterTableProxy(_globalParameters)

def mutate(*objects):		# TODO update syntax
	"""Function implementing the mutate statement."""
	if evaluatingRequirement:
		raise RuntimeParseError('used mutate statement inside a requirement')
	if len(objects) == 0:
		objects = currentScenario._objects
	for obj in objects:
		if not isinstance(obj, Object):
			raise RuntimeParseError('"mutate X" with X not an object')
		obj.mutationEnabled = True

### Prefix operators

def Visible(region):
	"""The 'visible <region>' operator."""
	region = toType(region, Region, '"visible X" with X not a Region')
	return region.intersect(ego().visibleRegion)

def NotVisible(region):
	"""The 'not visible <region>' operator."""
	region = toType(region, Region, '"not visible X" with X not a Region')
	return region.difference(ego().visibleRegion)

# front of <object>, etc.
ops = (
	'front', 'back', 'left', 'right',
	'front left', 'front right',
	'back left', 'back right'
)
template = '''\
def {function}(X):
	"""The '{syntax} of <object>' operator."""
	if not isinstance(X, Object):
		raise RuntimeParseError('"{syntax} of X" with X not an Object')
	return X.{property}
'''
for op in ops:
	func = ''.join(word.capitalize() for word in op.split(' '))
	prop = func[0].lower() + func[1:]
	definition = template.format(function=func, syntax=op, property=prop)
	exec(definition)

### Infix operators

def FieldAt(X, Y):
	"""The '<VectorField> at <vector>' operator."""
	if not isinstance(X, VectorField):
		raise RuntimeParseError('"X at Y" with X not a vector field')
	Y = toVector(Y, '"X at Y" with Y not a vector')
	return X[Y]

def RelativeTo(X, Y):
	"""The 'X relative to Y' polymorphic operator.

	Allowed forms:
		F relative to G (with at least one a field, the other a field or heading)
		<vector> relative to <oriented point> (and vice versa)
		<vector> relative to <vector>
		<heading> relative to <heading>
	"""
	xf, yf = isA(X, VectorField), isA(Y, VectorField)
	if xf or yf:
		if xf and yf and X.valueType != Y.valueType:
			raise RuntimeParseError('"X relative to Y" with X, Y fields of different types')
		fieldType = X.valueType if xf else Y.valueType
		error = '"X relative to Y" with field and value of different types'
		def helper(context):
			pos = context.position.toVector()
			xp = X[pos] if xf else toType(X, fieldType, error)
			yp = Y[pos] if yf else toType(Y, fieldType, error)
			return xp + yp
		return DelayedArgument({'position'}, helper)
	else:
		if isinstance(X, OrientedPoint):	# TODO too strict?
			if isinstance(Y, OrientedPoint):
				raise RuntimeParseError('"X relative to Y" with X, Y both oriented points')
			Y = toVector(Y, '"X relative to Y" with X an oriented point but Y not a vector')
			return X.relativize(Y)
		elif isinstance(Y, OrientedPoint):
			X = toVector(X, '"X relative to Y" with Y an oriented point but X not a vector')
			return Y.relativize(X)
		else:
			X = toTypes(X, (Vector, float), '"X relative to Y" with X neither a vector nor scalar')
			Y = toTypes(Y, (Vector, float), '"X relative to Y" with Y neither a vector nor scalar')
			return evaluateRequiringEqualTypes(lambda: X + Y, X, Y,
											   '"X relative to Y" with vector and scalar')

def OffsetAlong(X, H, Y):
	"""The 'X offset along H by Y' polymorphic operator.

	Allowed forms:
		<vector> offset along <heading> by <vector>
		<vector> offset along <field> by <vector>
	"""
	X = toVector(X, '"X offset along H by Y" with X not a vector')
	Y = toVector(Y, '"X offset along H by Y" with Y not a vector')
	if isinstance(H, VectorField):
		H = H[X]
	H = toHeading(H, '"X offset along H by Y" with H not a heading or vector field')
	return X.offsetRotated(H, Y)

def RelativePosition(X, Y=None):
	"""The 'relative position of <vector> [from <vector>]' operator.

	If the 'from <vector>' is omitted, the position of ego is used.
	"""
	X = toVector(X, '"relative position of X from Y" with X not a vector')
	if Y is None:
		Y = ego()
	Y = toVector(Y, '"relative position of X from Y" with Y not a vector')
	return X - Y

def RelativeHeading(X, Y=None):
	"""The 'relative heading of <heading> [from <heading>]' operator.

	If the 'from <heading>' is omitted, the heading of ego is used.
	"""
	X = toHeading(X, '"relative heading of X from Y" with X not a heading')
	if Y is None:
		Y = ego().heading
	else:
		Y = toHeading(Y, '"relative heading of X from Y" with Y not a heading')
	return normalizeAngle(X - Y)

def ApparentHeading(X, Y=None):
	"""The 'apparent heading of <oriented point> [from <vector>]' operator.

	If the 'from <vector>' is omitted, the position of ego is used.
	"""
	if not isinstance(X, OrientedPoint):
		raise RuntimeParseError('"apparent heading of X from Y" with X not an OrientedPoint')
	if Y is None:
		Y = ego()
	Y = toVector(Y, '"relative heading of X from Y" with Y not a vector')
	return apparentHeadingAtPoint(X.position, X.heading, Y)

def DistanceFrom(X, Y=None):
	"""The ``distance from {X} to {Y}`` polymorphic operator.

	Allowed forms:

	* ``distance from`` <vector> [``to`` <vector>]
	* ``distance from`` <region> [``to`` <vector>]
	* ``distance from`` <vector> ``to`` <region>

	If the ``to <vector>`` is omitted, the position of ego is used.
	"""
	X = toTypes(X, (Vector, Region), '"distance from X to Y" with X neither a vector nor region')
	if Y is None:
		Y = ego()
	Y = toTypes(Y, (Vector, Region), '"distance from X to Y" with Y neither a vector nor region')
	return X.distanceTo(Y)

def DistancePast(X, Y=None):
	"""The :samp:`distance past {vector} of {OP}` operator.

	If the :samp:`of {OP}` is omitted, the ego object is used.
	"""
	X = toVector(X, '"distance past X" with X not a vector')
	if Y is None:
		Y = ego()
	Y = toType(Y, OrientedPoint, '"distance past X of Y" with Y not an OrientedPoint')
	return Y.distancePast(X)

def AngleTo(X):
	"""The 'angle to <vector>' operator (using the position of ego as the reference)."""
	X = toVector(X, '"angle to X" with X not a vector')
	return ego().angleTo(X)

def AngleFrom(X, Y):
	"""The 'angle from <vector> to <vector>' operator."""
	X = toVector(X, '"angle from X to Y" with X not a vector')
	Y = toVector(Y, '"angle from X to Y" with Y not a vector')
	return X.angleTo(Y)

def Follow(F, X, D):
	"""The 'follow <field> from <vector> for <number>' operator."""
	if not isinstance(F, VectorField):
		raise RuntimeParseError('"follow F from X for D" with F not a vector field')
	X = toVector(X, '"follow F from X for D" with X not a vector')
	D = toScalar(D, '"follow F from X for D" with D not a number')
	pos = F.followFrom(X, D)
	heading = F[pos]
	return OrientedPoint(position=pos, heading=heading)

def CanSee(X, Y):
	"""The 'X can see Y' polymorphic operator.

	Allowed forms:
		<point> can see <object>
		<point> can see <vector>
	"""
	if not isinstance(X, Point):
		raise RuntimeParseError('"X can see Y" with X not a Point')
	if isinstance(Y, Point):
		return X.canSee(Y)
	else:
		Y = toVector(Y, '"X can see Y" with Y not a vector')
		return X.visibleRegion.containsPoint(Y)

### Specifiers

def With(prop, val):
	"""The 'with <property> <value>' specifier.

	Specifies the given property, with no dependencies.
	"""
	return Specifier(prop, val)

def At(pos):
	"""The 'at <vector>' specifier.

	Specifies 'position', with no dependencies."""
	pos = toVector(pos, 'specifier "at X" with X not a vector')
	return Specifier('position', pos)

def In(region):
	"""The 'in/on <region>' specifier.

	Specifies 'position', with no dependencies. Optionally specifies 'heading'
	if the given Region has a preferred orientation.
	"""
	region = toType(region, Region, 'specifier "in/on R" with R not a Region')
	extras = {'heading'} if alwaysProvidesOrientation(region) else {}
	return Specifier('position', Region.uniformPointIn(region), optionals=extras)

def alwaysProvidesOrientation(region):
	"""Whether a Region or distribution over Regions always provides an orientation."""
	if isinstance(region, Region):
		return region.orientation is not None
	elif (isinstance(region, Options)
		  and all(alwaysProvidesOrientation(opt) for opt in region.options)):
		return True
	else:	# TODO improve somehow!
		try:
			sample = region.sample()
			return sample.orientation is not None or sample is nowhere
		except RejectionException:
			return False

def Beyond(pos, offset, fromPt=None):
	"""The 'beyond X by Y [from Z]' polymorphic specifier.

	Specifies 'position', with no dependencies.

	Allowed forms:
		beyond <vector> by <number> [from <vector>]
		beyond <vector> by <vector> [from <vector>]

	If the 'from <vector>' is omitted, the position of ego is used.
	"""
	pos = toVector(pos, 'specifier "beyond X by Y" with X not a vector')
	dType = underlyingType(offset)
	if dType is float or dType is int:
		offset = Vector(0, offset)
	elif dType is not Vector:
		raise RuntimeParseError('specifier "beyond X by Y" with Y not a number or vector')
	if fromPt is None:
		fromPt = ego()
	fromPt = toVector(fromPt, 'specifier "beyond X by Y from Z" with Z not a vector')
	lineOfSight = fromPt.angleTo(pos)
	return Specifier('position', pos.offsetRotated(lineOfSight, offset))

def VisibleFrom(base):
	"""The 'visible from <Point>' specifier.

	Specifies 'position', with no dependencies.

	This uses the given object's 'visibleRegion' property, and so correctly
	handles the view regions of Points, OrientedPoints, and Objects.
	"""
	if not isinstance(base, Point):
		raise RuntimeParseError('specifier "visible from O" with O not a Point')
	return Specifier('position', Region.uniformPointIn(base.visibleRegion))

def VisibleSpec():
	"""The 'visible' specifier (equivalent to 'visible from ego').

	Specifies 'position', with no dependencies.
	"""
	return VisibleFrom(ego())

def OffsetBy(offset):
	"""The 'offset by <vector>' specifier.

	Specifies 'position', with no dependencies.
	"""
	offset = toVector(offset, 'specifier "offset by X" with X not a vector')
	pos = RelativeTo(offset, ego()).toVector()
	return Specifier('position', pos)

def OffsetAlongSpec(direction, offset):
	"""The 'offset along X by Y' polymorphic specifier.

	Specifies 'position', with no dependencies.

	Allowed forms:
		offset along <heading> by <vector>
		offset along <field> by <vector>
	"""
	return Specifier('position', OffsetAlong(ego(), direction, offset))

def Facing(heading):
	"""The 'facing X' polymorphic specifier.

	Specifies 'heading', with dependencies depending on the form:
		facing <number> -- no dependencies;
		facing <field> -- depends on 'position'.
	"""
	if isinstance(heading, VectorField):
		return Specifier('heading', DelayedArgument({'position'},
													lambda self: heading[self.position]))
	else:
		heading = toHeading(heading, 'specifier "facing X" with X not a heading or vector field')
		return Specifier('heading', heading)

def FacingToward(pos):
	"""The 'facing toward <vector>' specifier.

	Specifies 'heading', depending on 'position'.
	"""
	pos = toVector(pos, 'specifier "facing toward X" with X not a vector')
	return Specifier('heading', DelayedArgument({'position'},
												lambda self: self.position.angleTo(pos)))

def ApparentlyFacing(heading, fromPt=None):
	"""The 'apparently facing <heading> [from <vector>]' specifier.

	Specifies 'heading', depending on 'position'.

	If the 'from <vector>' is omitted, the position of ego is used.
	"""
	heading = toHeading(heading, 'specifier "apparently facing X" with X not a heading')
	if fromPt is None:
		fromPt = ego()
	fromPt = toVector(fromPt, 'specifier "apparently facing X from Y" with Y not a vector')
	value = lambda self: fromPt.angleTo(self.position) + heading
	return Specifier('heading', DelayedArgument({'position'}, value))

def LeftSpec(pos, dist=0):
	"""The 'left of X [by Y]' polymorphic specifier.

	Specifies 'position', depending on 'width'. See other dependencies below.

	Allowed forms:
		left of <oriented point> [by <scalar/vector>] -- optionally specifies 'heading';
		left of <vector> [by <scalar/vector>] -- depends on 'heading'.

	If the 'by <scalar/vector>' is omitted, zero is used.
	"""
	return leftSpecHelper('left of', pos, dist, 'width', lambda dist: (dist, 0),
						  lambda self, dx, dy: Vector(-self.width / 2 - dx, dy))

def RightSpec(pos, dist=0):
	"""The 'right of X [by Y]' polymorphic specifier.

	Specifies 'position', depending on 'width'. See other dependencies below.

	Allowed forms:
		right of <oriented point> [by <scalar/vector>] -- optionally specifies 'heading';
		right of <vector> [by <scalar/vector>] -- depends on 'heading'.

	If the 'by <scalar/vector>' is omitted, zero is used.
	"""
	return leftSpecHelper('right of', pos, dist, 'width', lambda dist: (dist, 0),
						  lambda self, dx, dy: Vector(self.width / 2 + dx, dy))

def Ahead(pos, dist=0):
	"""The 'ahead of X [by Y]' polymorphic specifier.

	Specifies 'position', depending on 'length'. See other dependencies below.

	Allowed forms:

	* ``ahead of`` <oriented point> [``by`` <scalar/vector>] -- optionally specifies 'heading';
	* ``ahead of`` <vector> [``by`` <scalar/vector>] -- depends on 'heading'.

	If the 'by <scalar/vector>' is omitted, zero is used.
	"""
	return leftSpecHelper('ahead of', pos, dist, 'length', lambda dist: (0, dist),
						  lambda self, dx, dy: Vector(dx, self.length / 2 + dy))

def Behind(pos, dist=0):
	"""The 'behind X [by Y]' polymorphic specifier.

	Specifies 'position', depending on 'length'. See other dependencies below.

	Allowed forms:
		behind <oriented point> [by <scalar/vector>] -- optionally specifies 'heading';
		behind <vector> [by <scalar/vector>] -- depends on 'heading'.

	If the 'by <scalar/vector>' is omitted, zero is used.
	"""
	return leftSpecHelper('behind', pos, dist, 'length', lambda dist: (0, dist),
						  lambda self, dx, dy: Vector(dx, -self.length / 2 - dy))

def leftSpecHelper(syntax, pos, dist, axis, toComponents, makeOffset):
	extras = set()
	if canCoerce(dist, float):
		dx, dy = toComponents(coerce(dist, float))
	elif canCoerce(dist, Vector):
		dx, dy = coerce(dist, Vector)
	else:
		raise RuntimeParseError(f'"{syntax} X by D" with D not a number or vector')
	if isinstance(pos, OrientedPoint):		# TODO too strict?
		val = lambda self: pos.relativize(makeOffset(self, dx, dy))
		new = DelayedArgument({axis}, val)
		extras.add('heading')
	else:
		pos = toVector(pos, f'specifier "{syntax} X" with X not a vector')
		val = lambda self: pos.offsetRotated(self.heading, makeOffset(self, dx, dy))
		new = DelayedArgument({axis, 'heading'}, val)
	return Specifier('position', new, optionals=extras)

def Following(field, dist, fromPt=None):
	"""The 'following F [from X] for D' specifier.

	Specifies 'position', and optionally 'heading', with no dependencies.

	Allowed forms:
		following <field> [from <vector>] for <number>

	If the 'from <vector>' is omitted, the position of ego is used.
	"""
	if fromPt is None:
		fromPt = ego()
	else:
		dist, fromPt = fromPt, dist
	if not isinstance(field, VectorField):
		raise RuntimeParseError('"following F" specifier with F not a vector field')
	fromPt = toVector(fromPt, '"following F from X for D" with X not a vector')
	dist = toScalar(dist, '"following F for D" with D not a number')
	pos = field.followFrom(fromPt, dist)
	heading = field[pos]
	val = OrientedVector.make(pos, heading)
	return Specifier('position', val, optionals={'heading'})

### Primitive functions overriding Python builtins

@distributionFunction
def filter(function, iterable):
	return list(builtins.filter(function, iterable))

@distributionFunction
def str(*args, **kwargs):
	return builtins.str(*args, **kwargs)
