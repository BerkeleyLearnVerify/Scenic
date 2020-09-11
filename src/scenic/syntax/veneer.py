
"""Python implementations of Scenic language constructs.

This module is automatically imported by all Scenic programs. In addition to
defining the built-in functions, operators, specifiers, etc., it also stores
global state such as the list of all created Scenic objects.
"""

__all__ = (
	# Primitive statements and functions
	'ego', 'require', 'resample', 'param', 'globalParameters', 'mutate', 'verbosePrint',
	'localPath', 'model', 'simulator', 'simulation', 'require_always', 'terminate_when',
	'terminate_simulation_when', 'in_initial_scenario',
	'sin', 'cos', 'hypot', 'max', 'min',
	'filter',
	# Prefix operators
	'Visible', 'NotVisible',
	'Front', 'Back', 'Left', 'Right',
	'FrontLeft', 'FrontRight', 'BackLeft', 'BackRight',
	# Infix operators
	'FieldAt', 'RelativeTo', 'OffsetAlong', 'RelativePosition',
	'RelativeHeading', 'ApparentHeading',
	'DistanceFrom', 'AngleTo', 'AngleFrom', 'Follow', 'CanSee',
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

# everything that should not be directly accessible from the language is imported here:
import builtins
import collections.abc
import importlib
import inspect
import sys
import random
import enum
import os.path
import traceback
import types
import typing
import itertools
from collections import defaultdict
from scenic.core.distributions import (Samplable, RejectionException, Distribution,
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
from scenic.core.simulators import (RejectSimulationException, EndSimulationAction,
                                    EndScenarioAction)
from scenic.core.utils import argsToString
from scenic.core.workspaces import Workspace
from scenic.core.scenarios import Scenario
import scenic.syntax.relations as relations

### Internals

activity = 0
currentScenario = None
scenarioStack = []
scenarios = []
evaluatingRequirement = False
_globalParameters = {}
lockedParameters = set()
lockedModel = None
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
	Scenic modules."""
	return activity > 0

def activate(paramOverrides={}, modelOverride=None, filename=None):
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
	newScenario = DynamicScenario._dummy(filename)	# placeholder scenario for top-level code
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

# External parameter creation

def registerExternalParameter(value):
	"""Register a parameter whose value is given by an external sampler."""
	if activity > 0:
		assert isinstance(value, ExternalParameter)
		externalParameters.append(value)

# Function call support

def wrapStarredValue(value, lineno):
	if isinstance(value, TupleDistribution) or not needsSampling(value):
		return value
	elif isinstance(value, Distribution):
		return [StarredDistribution(value, lineno)]
	else:
		raise RuntimeParseError(f'iterable unpacking cannot be applied to {value}')

def callWithStarArgs(func, /, *args, **kwargs):
	if not canUnpackDistributions(func):
		# wrap function to delay evaluation until starred distributions are sampled
		func = distributionFunction(func)
	return func(*args, **kwargs)

# Simulations

def beginSimulation(sim):
	global currentSimulation, currentScenario, inInitialScenario, runningScenarios
	global _globalParameters
	if isActive():
		raise RuntimeError('tried to start simulation during Scenic compilation!')
	assert currentSimulation is None
	assert currentScenario is None
	assert not scenarioStack
	currentSimulation = sim
	inInitialScenario = True
	currentScenario = sim.scene.dynamicScenario
	runningScenarios = {currentScenario}
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

@enum.unique
class RequirementType(enum.Enum):
	# requirements which must hold during initial sampling
	require = 'require'
	requireAlways = 'require always'

	# requirements used only during simulation
	terminateWhen = 'terminate when'
	terminateSimulationWhen = 'terminate simulation when'

	@property
	def constrainsSampling(self):
		return self is not self.terminateWhen

class PendingRequirement:
	def __init__(self, ty, condition, line, prob):
		self.ty = ty
		self.condition = condition
		self.line = line
		self.prob = prob

		# the translator wrapped the requirement in a lambda to prevent evaluation,
		# so we need to save the current values of all referenced names; we save
		# the ego object too since it can be referred to implicitly
		self.bindings = getAllGlobals(condition)
		self.egoObject = currentScenario._ego

	def compile(self, namespace, scenario, syntax=None):
		"""Create a closure testing the requirement in the correct runtime state.

		While we're at it, determine whether the requirement implies any relations
		we can use for pruning, and gather all of its dependencies.
		"""
		bindings, ego, line = self.bindings, self.egoObject, self.line
		condition = self.condition

		# Check whether requirement implies any relations used for pruning
		if self.ty.constrainsSampling and syntax:
			relations.inferRelationsFrom(syntax, bindings, ego, line)

		# Gather dependencies of the requirement
		deps = set()
		for value in bindings.values():
			if needsSampling(value):
				deps.add(value)
			if needsLazyEvaluation(value):
				raise InvalidScenarioError(f'requirement on line {line} uses value {value}'
										   ' undefined outside of object definition')
		if ego is not None:
			assert isinstance(ego, Samplable)
			deps.add(ego)

		# Construct closure
		def closure(values):
			global evaluatingRequirement, currentScenario
			# rebind any names referring to sampled objects
			for name, value in bindings.items():
				if value in values:
					namespace[name] = values[value]
			# evaluate requirement condition, reporting errors on the correct line
			if currentScenario is None:
				currentScenario = scenario
				clearScenario = True
			else:
				assert currentScenario is scenario
				clearScenario = False
			try:
				evaluatingRequirement = True
				# rebind ego object, which can be referred to implicitly
				oldEgo = currentScenario._ego
				if ego is not None:
					currentScenario._ego = values[ego]
				result = condition()
				assert not needsSampling(result)
				if needsLazyEvaluation(result):
					raise InvalidScenarioError(f'requirement on line {line} uses value'
											   ' undefined outside of object definition')
			finally:
				evaluatingRequirement = False
				currentScenario._ego = oldEgo
				if clearScenario:
					currentScenario = None
			return result

		return CompiledRequirement(self, closure, deps)

def getAllGlobals(req, restrictTo=None):
	"""Find all names the given lambda depends on, along with their current bindings."""
	namespace = req.__globals__
	if restrictTo is not None and restrictTo is not namespace:
		return {}
	externals = inspect.getclosurevars(req)
	assert not externals.nonlocals		# TODO handle these
	globs = dict(externals.builtins)
	for name, value in externals.globals.items():
		globs[name] = value
		if inspect.isfunction(value):
			subglobs = getAllGlobals(value, restrictTo=namespace)
			for name, value in subglobs.items():
				if name in globs:
					assert value is globs[name]
				else:
					globs[name] = value
	return globs

class CompiledRequirement:
	def __init__(self, pendingReq, closure, dependencies):
		self.ty = pendingReq.ty
		self.closure = closure
		self.line = pendingReq.line
		self.prob = pendingReq.prob
		self.dependencies = dependencies

	@property
	def constrainsSampling(self):
		return self.ty.constrainsSampling

	def satisfiedBy(self, sample):
		return self.closure(sample)

class BoundRequirement:
	def __init__(self, compiledReq, sample):
		self.ty = compiledReq.ty
		self.closure = compiledReq.closure
		self.line = compiledReq.line
		assert compiledReq.prob == 1
		self.sample = sample

	def isTrue(self):
		return self.closure(self.sample)

	def __str__(self):
		return f'"{self.ty.value}" on line {self.line}'

class DynamicRequirement:
	def __init__(self, ty, condition, line):
		self.ty = ty
		self.line = line

		global currentScenario
		scenario = currentScenario
		def closure():
			global currentScenario
			oldScenario = currentScenario
			currentScenario = scenario
			try:
				return condition()
			finally:
				currentScenario = oldScenario
		self.closure = closure

	def isTrue(self):
		return self.closure()

	def __str__(self):
		return f'"{self.ty.value}" on line {self.line}'

# Scenarios

class Invocable:
	def __init__(self, *args, **kwargs):
		if evaluatingGuard:
			raise RuntimeParseError(
				'tried to invoke behavior/scenario from inside guard or interrupt condition')
		self._runningIterator = None

	def _invokeSubBehavior(self, agent, subs, modifier=None):
		if modifier:
			if modifier.name == 'for':	# do X for Y [seconds | steps]
				timeLimit = modifier.value
				if not isinstance(timeLimit, (float, int)):
					raise RuntimeParseError('"do X for Y" with Y not a number')
				assert modifier.terminator in (None, 'seconds', 'steps')
				if modifier.terminator != 'steps':
					timeLimit /= currentSimulation.timestep
				startTime = currentSimulation.currentTime
				condition = lambda: currentSimulation.currentTime - startTime >= timeLimit
			elif modifier.name == 'until':	# do X until Y
				condition = modifier.value
			else:
				raise RuntimeError(f'internal parsing error: impossible modifier {modifier}')

			def body(behavior, agent):
				yield from self._invokeInner(agent, subs)
			handler = lambda behavior, agent: BlockConclusion.ABORT
			yield from runTryInterrupt(self, agent, body, [condition], [handler])
		else:
			yield from self._invokeInner(agent, subs)

	def _invokeInner(self, agent, subs):
		raise NotImplementedError

class DynamicScenario(Invocable):
	def __init_subclass__(cls, *args, **kwargs):
		scenarios.append(cls)

	_requirementSyntax = None 	# overridden by subclasses
	_simulatorFactory = None

	def __init__(self, *args, **kwargs):
		super().__init__()
		self._args = args
		self._kwargs = kwargs

		self._ego = None
		self._objects = []		# ordered for reproducibility
		self._globalParameters = {}
		self._externalParameters = []
		self._pendingRequirements = defaultdict(list)
		self._requirements = []
		self._requirementDeps = set()	# things needing to be sampled to evaluate the requirements

		self._agents = []
		self._monitors = []
		self._behaviors = []
		self._alwaysRequirements = []
		self._terminationConditions = []
		self._terminateSimulationConditions = []

		self._subScenarios = []
		self._endWithBehaviors = False
		self._prepared = False

	@classmethod
	def _dummy(cls, filename):
		scenario = cls()
		scenario._setup = None
		scenario._compose = None
		scenario._filename = filename 	# for debugging
		scenario._prepared = True
		return scenario

	@property
	def ego(self):
		return self._ego

	@property
	def objects(self):
		return tuple(self._objects)

	def _bindTo(self, scene):
		self._ego = scene.egoObject
		self._objects = list(scene.objects)
		self._agents = [obj for obj in scene.objects if obj.behavior is not None]
		self._alwaysRequirements = scene.alwaysRequirements
		self._terminationConditions = scene.terminationConditions
		self._terminateSimulationConditions = scene.terminateSimulationConditions

	def _prepare(self):
		assert not self._prepared
		self._prepared = True
		if currentSimulation:
			verbosePrint(f'Starting scenario {self}', level=3)

		global currentScenario, inInitialScenario
		oldScenario = currentScenario
		if oldScenario is not None:
			self._ego = oldScenario._ego	# inherit ego from parent
		currentScenario = self
		try:
			# Execute setup block
			if self._setup is not None:
				locs = self._setup(self, None, *self._args, **self._kwargs)
				self.__dict__.update(locs)	# save locals as if they were in class scope
				inInitialScenario = False

			# Start compose block
			if self._compose is not None:
				if not inspect.isgeneratorfunction(self._compose):
					from scenic.syntax.translator import composeBlock
					raise RuntimeParseError(f'"{composeBlock}" does not invoke any scenarios')
				self._runningIterator = self._compose(self, None, *self._args, **self._kwargs)
		finally:
			currentScenario = oldScenario

	def _start(self):
		assert self._prepared

		global currentScenario
		oldScenario = currentScenario
		currentScenario = self
		runningScenarios.add(self)
		try:
			# Initialize behavior coroutines of agents
			for agent in self._agents:
				assert isinstance(agent.behavior, Behavior), agent.behavior
				agent.behavior.start(agent)
			# Initialize monitor coroutines
			for monitor in self._monitors:
				monitor.start()
		finally:
			currentScenario = oldScenario

	def _step(self):
		global currentScenario
		oldScenario = currentScenario

		# Execute compose block, if any
		composeDone = False
		if self._runningIterator is None:
			composeDone = True		# compose block ended in an earlier step
		else:
			currentScenario = self
			try:
				result = self._runningIterator.send(None)
				if isinstance(result, (EndSimulationAction, EndScenarioAction)):
					return self._stop(result)
			except StopIteration:
				self._runningIterator = None
				composeDone = True
			finally:
				currentScenario = oldScenario

		# If there is a compose block and it has finished, we're done
		if self._compose is not None and composeDone:
			return self._stop('finished compose block')

		# Optionally end when all our agents' behaviors have ended
		if self._endWithBehaviors:
			if all(agent.behavior._isFinished for agent in self._agents):
				return self._stop('all behaviors finished')

		# Otherwise, check if any termination conditions apply
		for req in self._terminationConditions:
			if req.isTrue():
				return self._stop(req)

		# Scenario will not terminate yet
		return None

	def _stop(self, reason):
		verbosePrint(f'Stopping scenario {self} because of: {reason}', level=3)
		runningScenarios.remove(self)
		self._runningIterator = None
		return reason

	def _invokeInner(self, agent, subs):
		for sub in subs:
			if not isinstance(sub, DynamicScenario):
				raise RuntimeParseError(f'expected a scenario, got {sub}')
			sub._prepare()
			sub._start()
		self._subScenarios = list(subs)
		while True:
			newSubs = []
			for sub in self._subScenarios:
				terminationReason = sub._step()
				if isinstance(terminationReason, EndSimulationAction):
					yield terminationReason
				elif terminationReason is None:
					newSubs.append(sub)
			self._subScenarios = newSubs
			if not newSubs:
				return
			yield None

	def _checkAlwaysRequirements(self):
		for req in self._alwaysRequirements:
			if not req.isTrue():
				# always requirements should never be violated at time 0, since
				# they are enforced during scene sampling
				assert currentSimulation.currentTime > 0
				raise RejectSimulationException(str(req))
		for sub in self._subScenarios:
			sub._checkAlwaysRequirements()

	def _runMonitors(self):
		terminationReason = None
		for monitor in self._monitors:
			action = monitor.step()
			if isinstance(action, EndSimulationAction):
				terminationReason = action
				# do not exit early, since subsequent monitors could reject the simulation
		for sub in self._subScenarios:
			subreason = sub._runMonitors()
			if subreason is not None:
				terminationReason = subreason
		return terminationReason

	def _checkSimulationTerminationConditions(self):
		for req in self._terminateSimulationConditions:
			if req.isTrue():
				return req
		return None

	@property
	def _allAgents(self):
		agents = list(self._agents)
		for sub in self._subScenarios:
			agents.extend(sub._allAgents)
		return agents

	def _inherit(self, other):
		self._objects.extend(other._objects)
		self._agents.extend(other._agents)
		self._globalParameters.update(other._globalParameters)
		self._externalParameters.extend(other._externalParameters)
		self._requirements.extend(other._requirements)
		self._behaviors.extend(other._behaviors)

	def _registerObject(self, obj):
		self._objects.append(obj)
		if getattr(obj, 'behavior', None) is not None:
			self._agents.append(obj)
		if currentSimulation:
			currentSimulation.createObject(obj)

	def _compileRequirements(self, namespace):
		requirementSyntax = self._requirementSyntax
		assert requirementSyntax is not None
		for reqID, requirement in self._pendingRequirements.items():
			syntax = requirementSyntax[reqID] if requirementSyntax else None
			compiledReq = requirement.compile(namespace, self, syntax)

			self._addRequirement(compiledReq)
			self._requirementDeps.update(compiledReq.dependencies)

	def _addRequirement(self, req):
		if req.ty is RequirementType.require:
			place = self._requirements
		elif req.ty is RequirementType.requireAlways:
			place = self._alwaysRequirements
		elif req.ty is RequirementType.terminateWhen:
			place = self._terminationConditions
		elif req.ty is RequirementType.terminateSimulationWhen:
			place = self._terminateSimulationConditions
		else:
			raise RuntimeError(f'internal error: requirement {req} has unknown type!')
		place.append(req)

	def _toScenario(self, namespace):
		assert self._prepared

		if self._ego is None and self._compose is None:
			raise InvalidScenarioError('did not specify ego object')

		# Extract workspace, if one is specified
		if 'workspace' in namespace:
			workspace = namespace['workspace']
			if not isinstance(workspace, Workspace):
				raise InvalidScenarioError(f'workspace {workspace} is not a Workspace')
			if needsSampling(workspace):
				raise InvalidScenarioError('workspace must be a fixed region')
			if needsLazyEvaluation(workspace):
				raise InvalidScenarioError('workspace uses value undefined '
										   'outside of object definition')
		else:
			workspace = None

		scenario = Scenario(workspace, self._simulatorFactory,
							self._objects, self._ego,
							self._globalParameters, self._externalParameters,
							self._requirements, self._requirementDeps,
							self._monitors, self._behaviorNamespaces,
							self)	# TODO unify these!
		return scenario

	def __str__(self):
		args = argsToString(itertools.chain(self._args, self._kwargs.items()))
		return self.__class__.__name__ + args

# Behaviors

class Behavior(Invocable, Samplable):
	def __init_subclass__(cls):
		if cls.__module__ is not __name__:
			currentScenario._behaviors.append(cls)

	def __init__(self, *args, **kwargs):
		self.args = tuple(toDistribution(arg) for arg in args)
		self.kwargs = { name: toDistribution(arg) for name, arg in kwargs.items() }
		if not inspect.isgeneratorfunction(self.makeGenerator):
			raise RuntimeParseError(f'{self} does not take any actions'
			                        ' (perhaps you forgot to use "take" or "do"?)')

		# Validate arguments to the behavior
		sig = inspect.signature(self.makeGenerator)
		try:
			sig.bind(None, *args, **kwargs)
		except TypeError as e:
			raise RuntimeParseError(str(e)) from e
		Samplable.__init__(self, itertools.chain(self.args, self.kwargs.values()))
		Invocable.__init__(self)

	def sampleGiven(self, value):
		args = (value[arg] for arg in self.args)
		kwargs = { name: value[val] for name, val in self.kwargs.items() }
		return type(self)(*args, **kwargs)

	def start(self, agent):
		self.agent = agent
		self._runningIterator = self.makeGenerator(agent, *self.args, **self.kwargs)

	def step(self):
		assert self._runningIterator
		global currentBehavior
		oldBehavior = currentBehavior
		try:
			currentBehavior = self
			actions = self._runningIterator.send(None)
		except StopIteration:
			actions = ()	# behavior ended early
		finally:
			currentBehavior = oldBehavior
		return actions

	def stop(self):
		self.agent = None
		self._runningIterator = None

	@property
	def _isFinished(self):
		return self._runningIterator is None

	def _invokeInner(self, agent, subs):
		global currentBehavior
		assert len(subs) == 1
		sub = subs[0]
		if not isinstance(sub, Behavior):
			raise RuntimeParseError(f'expected a behavior, got {sub}')
		sub.start(agent)
		try:
			currentBehavior = sub
			yield from sub._runningIterator
		finally:
			sub.stop()
			currentBehavior = self

	def __str__(self):
		args = argsToString(itertools.chain(self.args, self.kwargs.items()))
		return self.__class__.__name__ + args

def makeTerminationAction(line):
	assert not isActive()
	return EndSimulationAction(line)

# Monitors

class Monitor(Behavior):
	def __init_subclass__(cls):
		super().__init_subclass__()
		currentScenario._monitors.append(cls())

	def start(self):
		return super().start(None)

monitorPrefix = '_Scenic_monitor_'
def functionForMonitor(name):
	return monitorPrefix + name
def isAMonitorName(name):
	return name.startswith(monitorPrefix)
def monitorName(name):
	return name[len(monitorPrefix):]

# Try-interrupt blocks

def runTryInterrupt(behavior, agent, body, conditions, handlers):
	body = InterruptBlock(None, body)
	interrupts = [InterruptBlock(c, h) for c, h in zip(conditions, handlers)]
	while True:
		# find next block to run, if any
		block = body
		for interrupt in interrupts:
			if interrupt.isEnabled or interrupt.isRunning:
				block = interrupt
				break
		result, concluded = block.step(behavior, agent)
		if concluded:
			if result is BlockConclusion.FINISHED and block is not body:
				continue 	# interrupt handler finished
			else:
				return result 	# entire try-interrupt statement will terminate
		else:
			yield result
			behavior.checkInvariants()

@enum.unique
class BlockConclusion(enum.Enum):
	FINISHED = enum.auto()
	ABORT = enum.auto()
	RETURN = enum.auto()
	BREAK = enum.auto()
	CONTINUE = enum.auto()

	def __call__(self, value):
		self.return_value = value
		return self

class InterruptBlock:
	def __init__(self, condition, body):
		self.condition = condition
		self.body = body
		self.runningIterator = None

	@property
	def isEnabled(self):
		global evaluatingGuard
		try:
			evaluatingGuard = True
			return bool(self.condition())
		finally:
			evaluatingGuard = False

	@property
	def isRunning(self):
		return self.runningIterator is not None

	def step(self, behavior, agent):
		if not self.runningIterator:
			it = self.body(behavior, agent)
			if not isinstance(it, types.GeneratorType):
				return (it, True)
			self.runningIterator = it
		try:
			result = self.runningIterator.send(None)
			return (result, False)
		except StopIteration as e:
			self.runningIterator = None
			return (e.value, True)

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

def require(reqID, req, line, prob=1):
	"""Function implementing the require statement."""
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
				raise RejectSimulationException(f'requirement on line {line}')
	else:	# requirement being defined at compile time
		assert reqID not in currentScenario._pendingRequirements
		preq = PendingRequirement(RequirementType.require, req, line, prob)
		currentScenario._pendingRequirements[reqID] = preq

def require_always(reqID, req, line):
	"""Function implementing the 'require always' statement."""
	makeRequirement(RequirementType.requireAlways, reqID, req, line)

def terminate_when(reqID, req, line):
	"""Function implementing the 'terminate when' statement."""
	makeRequirement(RequirementType.terminateWhen, reqID, req, line)

def terminate_simulation_when(reqID, req, line):
	"""Function implementing the 'terminate simulation when' statement."""
	makeRequirement(RequirementType.terminateSimulationWhen, reqID, req, line)

def makeRequirement(ty, reqID, req, line):
	if evaluatingRequirement:
		raise RuntimeParseError(f'tried to use "{ty.value}" inside a requirement')
	elif currentBehavior is not None:
		raise RuntimeParseError(f'"{ty.value}" inside a behavior on line {line}')
	elif currentSimulation is not None:
		assert ty is not RequirementType.require
		dreq = DynamicRequirement(ty, req, line)
		currentScenario._addRequirement(dreq)
	else:	# requirement being defined at compile time
		assert reqID not in currentScenario._pendingRequirements
		preq = PendingRequirement(ty, req, line, 1)
		currentScenario._pendingRequirements[reqID] = preq

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
	assert currentSimulation is not None
	return inInitialScenario

def model(namespace, modelName):
	if lockedModel is not None:
		modelName = lockedModel
	try:
		module = importlib.import_module(modelName)
	except ModuleNotFoundError:
		raise InvalidScenarioError(f'could not import world model {modelName}') from None
	names = module.__dict__.get('__all__', None)
	if names is not None:
		for name in names:
			namespace[name] = getattr(module, name)
	else:
		for name, value in module.__dict__.items():
			if not name.startswith('_'):
				namespace[name] = value

@distributionFunction
def filter(function, iterable):
	return list(builtins.filter(function, iterable))

def param(*quotedParams, **params):
	"""Function implementing the param statement."""
	if evaluatingRequirement:
		raise RuntimeParseError('tried to create a global parameter inside a requirement')
	for name, value in params.items():
		if name not in lockedParameters:
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

### Exceptions

class GuardViolation(Exception):
	"""Abstract exception raised when a guard of a behavior is violated.

	This will never be raised directly; either of the subclasses `PreconditionViolation`
	or `InvariantViolation` will be used, as appropriate.
	"""
	pass

class PreconditionViolation(GuardViolation):
	"""Raised when a precondition is violated when invoking a behavior."""
	pass

class InvariantViolation(GuardViolation):
	"""Raised when an invariant is violated when invoking/resuming a behavior."""
	pass

