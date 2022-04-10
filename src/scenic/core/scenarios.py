"""Scenario and scene objects."""

import random
import time

from scenic.core.distributions import (Samplable, ConstantSamplable, RejectionException,
                                       needsSampling)
from scenic.core.lazy_eval import needsLazyEvaluation
from scenic.core.external_params import ExternalSampler
from scenic.core.regions import EmptyRegion
from scenic.core.workspaces import Workspace
from scenic.core.vectors import Vector
from scenic.core.utils import areEquivalent
from scenic.core.errors import InvalidScenarioError, optionallyDebugRejection
from scenic.core.dynamics import Behavior
from scenic.core.requirements import BoundRequirement

class Scene:
	"""Scene()

	A scene generated from a Scenic scenario.

	To run a dynamic simulation from a scene, create an instance of `Simulator` for the
	simulator you want to use, and pass the scene to its `simulate` method.

	Attributes:
		objects (tuple of :obj:`~scenic.core.object_types.Object`): All objects in the
		  scene. The ``ego`` object is first.
		egoObject (:obj:`~scenic.core.object_types.Object`): The ``ego`` object.
		params (dict): Dictionary mapping the name of each global parameter to its value.
		workspace (:obj:`~scenic.core.workspaces.Workspace`): Workspace for the scenario.
	"""
	def __init__(self, workspace, objects, egoObject, params,
				 alwaysReqs=(), eventuallyReqs=(),
				 terminationConds=(), termSimulationConds=(),
				 recordedExprs=(), recordedInitialExprs=(), recordedFinalExprs=(),
				 monitors=(), behaviorNamespaces={}, dynamicScenario=None):
		self.workspace = workspace
		self.objects = tuple(objects)
		self.egoObject = egoObject
		self.params = params
		self.alwaysRequirements = tuple(alwaysReqs)
		self.eventuallyRequirements = tuple(eventuallyReqs)
		self.terminationConditions = tuple(terminationConds)
		self.terminateSimulationConditions = tuple(termSimulationConds)
		self.recordedExprs = tuple(recordedExprs)
		self.recordedInitialExprs = tuple(recordedInitialExprs)
		self.recordedFinalExprs = tuple(recordedFinalExprs)
		self.monitors = tuple(monitors)
		self.behaviorNamespaces = behaviorNamespaces
		self.dynamicScenario = dynamicScenario

	def show(self, zoom=None, block=True):
		"""Render a schematic of the scene for debugging."""
		import matplotlib.pyplot as plt
		plt.gca().set_aspect('equal')
		# display map
		self.workspace.show(plt)
		# draw objects
		for obj in self.objects:
			obj.show(self.workspace, plt, highlight=(obj is self.egoObject))
		# zoom in if requested
		if zoom != None:
			self.workspace.zoomAround(plt, self.objects, expansion=zoom)
		plt.show(block=block)

class Scenario:
	"""Scenario()

	A compiled Scenic scenario, from which scenes can be sampled.
	"""
	def __init__(self, workspace, simulator,
				 objects, egoObject,
				 params, externalParams,
				 requirements, requirementDeps,
				 monitors, behaviorNamespaces,
				 dynamicScenario):
		if workspace is None:
			workspace = Workspace()		# default empty workspace
		self.workspace = workspace
		self.simulator = simulator		# simulator for dynamic scenarios
		# make ego the first object, while otherwise preserving order
		ordered = []
		for obj in objects:
			if obj is not egoObject:
				ordered.append(obj)
		self.objects = (egoObject,) + tuple(ordered) if egoObject else tuple(ordered)
		self.egoObject = egoObject
		self.params = dict(params)
		self.externalParams = tuple(externalParams)
		self.externalSampler = ExternalSampler.forParameters(self.externalParams, self.params)
		self.monitors = tuple(monitors)
		self.behaviorNamespaces = behaviorNamespaces
		self.dynamicScenario = dynamicScenario

		staticReqs, alwaysReqs, terminationConds = [], [], []
		self.requirements = tuple(dynamicScenario._requirements)	# TODO clean up
		self.alwaysRequirements = tuple(dynamicScenario._alwaysRequirements)
		self.eventuallyRequirements = tuple(dynamicScenario._eventuallyRequirements)
		self.terminationConditions = tuple(dynamicScenario._terminationConditions)
		self.terminateSimulationConditions = tuple(dynamicScenario._terminateSimulationConditions)
		self.initialRequirements = self.requirements + self.alwaysRequirements
		assert all(req.constrainsSampling for req in self.initialRequirements)
		self.recordedExprs = tuple(dynamicScenario._recordedExprs)
		self.recordedInitialExprs = tuple(dynamicScenario._recordedInitialExprs)
		self.recordedFinalExprs = tuple(dynamicScenario._recordedFinalExprs)
		# dependencies must use fixed order for reproducibility
		paramDeps = tuple(p for p in self.params.values() if isinstance(p, Samplable))
		behaviorDeps = []
		for namespace in self.behaviorNamespaces.values():
			for value in namespace.values():
				if isinstance(value, Samplable):
					behaviorDeps.append(value)
		self.dependencies = self.objects + paramDeps + tuple(requirementDeps) + tuple(behaviorDeps)

		self.validate()

	def isEquivalentTo(self, other):
		if type(other) is not Scenario:
			return False
		return (areEquivalent(other.workspace, self.workspace)
			and areEquivalent(other.objects, self.objects)
			and areEquivalent(other.params, self.params)
			and areEquivalent(other.externalParams, self.externalParams)
			and areEquivalent(other.requirements, self.requirements)
			and other.externalSampler == self.externalSampler)

	def containerOfObject(self, obj):
		if hasattr(obj, 'regionContainedIn') and obj.regionContainedIn is not None:
			return obj.regionContainedIn
		else:
			return self.workspace.region

	def validate(self):
		"""Make some simple static checks for inconsistent built-in requirements.

		:meta private:
		"""
		objects = self.objects
		staticVisibility = self.egoObject and not needsSampling(self.egoObject.visibleRegion)
		staticBounds = [self.hasStaticBounds(obj) for obj in objects]
		for i in range(len(objects)):
			oi = objects[i]
			container = self.containerOfObject(oi)
			# Trivial case where container is empty
			if isinstance(container, EmptyRegion):
				raise InvalidScenarioError(f'Container region of {oi} is empty')
			# skip objects with unknown positions or bounding boxes
			if not staticBounds[i]:
				continue
			# Require object to be contained in the workspace/valid region
			if not needsSampling(container) and not container.containsObject(oi):
				raise InvalidScenarioError(f'Object at {oi.position} does not fit in container')
			# Require object to be visible from the ego object
			if staticVisibility and oi.requireVisible is True and oi is not self.egoObject:
				if not self.egoObject.canSee(oi):
					raise InvalidScenarioError(f'Object at {oi.position} is not visible from ego')
			if not oi.allowCollisions:
				# Require object to not intersect another object
				for j in range(i):
					oj = objects[j]
					if oj.allowCollisions or not staticBounds[j]:
						continue
					if oi.intersects(oj):
						raise InvalidScenarioError(f'Object at {oi.position} intersects'
												   f' object at {oj.position}')

	def hasStaticBounds(self, obj):
		if needsSampling(obj.position):
			return False
		if any(needsSampling(corner) for corner in obj.corners):
			return False
		return True

	def generate(self, maxIterations=2000, verbosity=0, feedback=None):
		"""Sample a `Scene` from this scenario.

		Args:
			maxIterations (int): Maximum number of rejection sampling iterations.
			verbosity (int): Verbosity level.
			feedback (float): Feedback to pass to external samplers doing active sampling.
				See :mod:`scenic.core.external_params`.

		Returns:
			A pair with the sampled `Scene` and the number of iterations used.

		Raises:
			`RejectionException`: if no valid sample is found in **maxIterations** iterations.
		"""
		objects = self.objects

		# choose which custom requirements will be enforced for this sample
		activeReqs = [req for req in self.initialRequirements if random.random() <= req.prob]

		# do rejection sampling until requirements are satisfied
		rejection = True
		iterations = 0
		while rejection is not None:
			if iterations > 0:	# rejected the last sample
				if verbosity >= 2:
					print(f'  Rejected sample {iterations} because of: {rejection}')
				if self.externalSampler is not None:
					feedback = self.externalSampler.rejectionFeedback
			if iterations >= maxIterations:
				raise RejectionException(f'failed to generate scenario in {iterations} iterations')
			iterations += 1
			try:
				if self.externalSampler is not None:
					self.externalSampler.sample(feedback)
				sample = Samplable.sampleAll(self.dependencies)
			except RejectionException as e:
				optionallyDebugRejection(e)
				rejection = e
				continue
			rejection = None
			ego = sample[self.egoObject]
			# Normalize types of some built-in properties
			for obj in objects:
				sampledObj = sample[obj]
				assert not needsSampling(sampledObj)
				# position, heading
				assert isinstance(sampledObj.position, Vector)
				sampledObj.heading = float(sampledObj.heading)
				# behavior
				behavior = sampledObj.behavior
				if behavior is not None and not isinstance(behavior, Behavior):
					raise InvalidScenarioError(
						f'behavior {behavior} of Object {obj} is not a behavior')

			# Check built-in requirements
			for i in range(len(objects)):
				vi = sample[objects[i]]
				# Require object to be contained in the workspace/valid region
				container = self.containerOfObject(vi)
				if not container.containsObject(vi):
					rejection = 'object containment'
					break
				# Require object to be visible from the ego object
				if vi.requireVisible and vi is not ego and not ego.canSee(vi):
					rejection = 'object visibility'
					break
				# Require object to not intersect another object
				if not vi.allowCollisions:
					for j in range(i):
						vj = sample[objects[j]]
						if not vj.allowCollisions and vi.intersects(vj):
							rejection = 'object intersection'
							break
				if rejection is not None:
					break
			if rejection is not None:
				optionallyDebugRejection()
				continue
			# Check user-specified requirements
			for req in activeReqs:
				if not req.satisfiedBy(sample):
					rejection = str(req)
					break

		# obtained a valid sample; assemble a scene from it
		sampledObjects = tuple(sample[obj] for obj in objects)
		sampledParams = {}
		for param, value in self.params.items():
			sampledValue = sample[value]
			assert not needsLazyEvaluation(sampledValue)
			sampledParams[param] = sampledValue
		sampledNamespaces = {}
		for modName, namespace in self.behaviorNamespaces.items():
			sampledNamespace = { name: sample[value] for name, value in namespace.items() }
			sampledNamespaces[modName] = (namespace, sampledNamespace, namespace.copy())
		alwaysReqs = (BoundRequirement(req, sample) for req in self.alwaysRequirements)
		eventuallyReqs = (BoundRequirement(req, sample) for req in self.eventuallyRequirements)
		terminationConds = (BoundRequirement(req, sample)
							for req in self.terminationConditions)
		termSimulationConds = (BoundRequirement(req, sample)
							   for req in self.terminateSimulationConditions)
		recordedExprs = (BoundRequirement(req, sample) for req in self.recordedExprs)
		recordedInitialExprs = (BoundRequirement(req, sample)
		                        for req in self.recordedInitialExprs)
		recordedFinalExprs = (BoundRequirement(req, sample)
		                      for req in self.recordedFinalExprs)
		scene = Scene(self.workspace, sampledObjects, ego, sampledParams,
					  alwaysReqs, eventuallyReqs, terminationConds, termSimulationConds,
					  recordedExprs, recordedInitialExprs,recordedFinalExprs,
					  self.monitors, sampledNamespaces, self.dynamicScenario)
		return scene, iterations

	def resetExternalSampler(self):
		"""Reset the scenario's external sampler, if any.

		If the Python random seed is reset before calling this function, this
		should cause the sequence of generated scenes to be deterministic.
		"""
		self.externalSampler = ExternalSampler.forParameters(self.externalParams, self.params)

	def conditionOn(self, scene=None, objects=(), params={}):
		"""Condition the scenario on particular values for some objects or parameters.

		This method changes the distribution of the scenario and should be used with
		care: it does not attempt to check that the new distribution is equivalent to the
		old one or that it has nonzero probability of satisfying the scenario's
		requirements.

		For example, to sample object #5 in the scenario once and then leave it fixed in
		all subsequent samples::

			sceneA, _ = scenario.generate()
			scenario.conditionOn(scene=sceneA, objects=(5,))
			sceneB, _ = scenario.generate()		# will have the same object 5 as sceneA

		Args:
			scene (Scene): Scene from which to take values for the given **objects**,
				if any.
			objects: Sequence of indices specifying which objects in this scenario should
				be conditioned on the corresponding objects in **scene** (i.e. those with
				the same index in the list of objects).
			params (dict): Dictionary of global parameters to condition and their new
				values (which may be constants or distributions).
		"""
		assert objects or params
		assert bool(scene) == bool(objects)
		if scene:
			assert len(self.objects) == len(scene.objects)
		for i in objects:
			assert i < len(self.objects)
			self.objects[i].conditionTo(scene.objects[i])
		for param, newVal in params.items():
			curVal = self.params[param]
			if isinstance(curVal, Samplable):
				if not isinstance(newVal, Samplable):
					newVal = ConstantSamplable(newVal)
				curVal.conditionTo(newVal)
			else:
				self.params[param] = newVal

	def getSimulator(self):
		if self.simulator is None:
			raise RuntimeError('scenario does not specify a simulator')
		import scenic.syntax.veneer as veneer
		return veneer.instantiateSimulator(self.simulator, self.params)
