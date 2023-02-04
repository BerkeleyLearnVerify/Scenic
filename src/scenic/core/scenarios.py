"""Scenario and scene objects."""

import random
import time

from scenic.core.distributions import Samplable, RejectionException, needsSampling, Options
from scenic.core.lazy_eval import needsLazyEvaluation
from scenic.core.external_params import ExternalSampler
from scenic.core.regions import EmptyRegion
from scenic.core.workspaces import Workspace
from scenic.core.vectors import Vector
from scenic.core.utils import areEquivalent
from scenic.core.errors import InvalidScenarioError
import scenic.syntax.veneer as veneer

class Scene:
	"""Scene()

	A scene generated from a Scenic scenario.

	Attributes:
		objects (tuple of :obj:`~scenic.core.object_types.Object`): All objects in the
		  scene. The ``ego`` object is first.
		egoObject (:obj:`~scenic.core.object_types.Object`): The ``ego`` object.
		params (dict): Dictionary mapping the name of each global parameter to its value.
		workspace (:obj:`~scenic.core.workspaces.Workspace`): Workspace for the scenario.
	"""
	def __init__(self, workspace, objects, egoObject, params,
				 alwaysReqs=(), terminationConds=(), termSimulationConds=(), monitors=(),
				 behaviorNamespaces={}, dynamicScenario=None):
		self.workspace = workspace
		self.objects = tuple(objects)
		self.egoObject = egoObject
		self.params = params
		self.alwaysRequirements = tuple(alwaysReqs)
		self.terminationConditions = tuple(terminationConds)
		self.terminateSimulationConditions = tuple(termSimulationConds)
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
		self.original_objects = objects
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
		self.terminationConditions = tuple(dynamicScenario._terminationConditions)
		self.terminateSimulationConditions = tuple(dynamicScenario._terminateSimulationConditions)
		self.initialRequirements = self.requirements + self.alwaysRequirements
		assert all(req.constrainsSampling for req in self.initialRequirements)
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
			# Require object to not intersect another object
			for j in range(i):
				oj = objects[j]
				if not staticBounds[j]:
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

	def generateForQuery(self, maxIterations=2000, verbosity=0, feedback=None):
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
		objects = self.original_objects

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
				if behavior is not None and not isinstance(behavior, veneer.Behavior):
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
				for j in range(i):
					vj = sample[objects[j]]
					if vi.intersects(vj):
						rejection = 'object intersection'
						break
				if rejection is not None:
					break
			if rejection is not None:
				continue
			# Check user-specified requirements
			for req in activeReqs:
				if not req.satisfiedBy(sample):
					rejection = f'user-specified requirement (line {req.line})'
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
		alwaysReqs = (veneer.BoundRequirement(req, sample) for req in self.alwaysRequirements)
		terminationConds = (veneer.BoundRequirement(req, sample)
							for req in self.terminationConditions)
		termSimulationConds = (veneer.BoundRequirement(req, sample)
							   for req in self.terminateSimulationConditions)
		scene = Scene(self.workspace, sampledObjects, ego, sampledParams,
					  alwaysReqs, terminationConds, termSimulationConds, self.monitors,
					  sampledNamespaces, self.dynamicScenario)
		return scene, iterations

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
				if behavior is not None and not isinstance(behavior, veneer.Behavior):
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
				for j in range(i):
					vj = sample[objects[j]]
					if vi.intersects(vj):
						rejection = 'object intersection'
						break
				if rejection is not None:
					break
			if rejection is not None:
				continue
			# Check user-specified requirements
			for req in activeReqs:
				if not req.satisfiedBy(sample):
					rejection = f'user-specified requirement (line {req.line})'
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
		alwaysReqs = (veneer.BoundRequirement(req, sample) for req in self.alwaysRequirements)
		terminationConds = (veneer.BoundRequirement(req, sample)
							for req in self.terminationConditions)
		termSimulationConds = (veneer.BoundRequirement(req, sample)
							   for req in self.terminateSimulationConditions)
		scene = Scene(self.workspace, sampledObjects, ego, sampledParams,
					  alwaysReqs, terminationConds, termSimulationConds, self.monitors,
					  sampledNamespaces, self.dynamicScenario)
		return scene, iterations

	def checkRequirements(self):
		sample = Samplable.sampleAll(self.dependencies)
		for req in self.initialRequirements:
			if not req.satisfiedBy(sample):
				return False
		return True

	def resetExternalSampler(self):
		"""Reset the scenario's external sampler, if any.

		If the Python random seed is reset before calling this function, this
		should cause the sequence of generated scenes to be deterministic."""
		self.externalSampler = ExternalSampler.forParameters(self.externalParams, self.params)

	def getSimulator(self):
		if self.simulator is None:
			raise RuntimeError('scenario does not specify a simulator')
		try:
			assert not veneer._globalParameters		# TODO improve hack!
			veneer._globalParameters = dict(self.params)
			return self.simulator()
		finally:
			veneer._globalParameters = {}

	## Dependency Analysis
	def cacheDependencies(self, obj, depSet=None):
		if depSet is None:
			depSet = set()
	    
		depSet.add(obj)
		if not isinstance(obj._conditioned, Samplable):
			return depSet

		for dep in obj._conditioned._dependencies:
			if isinstance(dep._conditioned, Samplable):
				depSet = self.cacheDependencies(dep, depSet)

		return depSet

	def addToFeatureDict(self, feature, featureDict, name):
		featureDict[name] = {}
		featureDict[name]['cachedDep'] = self.cacheDependencies(feature)
		featureDict[name]['dependent_features'] = set()
		featureDict[name]['jointly_dependent_features'] = set()
		featureDict[name]['feature'] = feature
		return featureDict

	def dependencyAnalysis(self, debug=False):
	## TODO: also account for out of order object specification in the program

		# cache features' dependencies
		featureDict = {}
		cached_featureList = []
		for i in range(len(self.objects)):
			obj = self.objects[i]
			obj_name = 'obj'+str(i)
			obj_pos = obj_name+'_position'
			obj_heading = obj_name+'_heading'
			pos_feature = obj.position
			heading_feature = obj.heading
			cached_featureList.extend([obj_pos, obj_heading])
			self.addToFeatureDict(pos_feature, featureDict, obj_pos)
			self.addToFeatureDict(heading_feature,  featureDict, obj_heading)
			pos_feature._conditioned = 0
			heading_feature._conditioned = 0
            
		for i in range(len(cached_featureList)):
			for j in range(len(cached_featureList)):
				if i >= j: # same feature or already checked
					continue
				else:
					feature1 = cached_featureList[i]
					feature2 = cached_featureList[j]
					feature1_dict = featureDict[feature1]
					feature2_dict = featureDict[feature2]

					feature1_cacheDep = feature1_dict['cachedDep']
					feature2_cacheDep = feature2_dict['cachedDep']
					shared_dependencies = feature1_cacheDep.intersection(feature2_cacheDep)
		            
					if len(shared_dependencies) > 0:
						# two features cannot be both dependent and jointly dependent
						if featureDict[feature1]['feature'] in shared_dependencies:
							featureDict[feature2]['dependent_features'].add(feature1)
						elif featureDict[feature2]['feature'] in shared_dependencies:
							featureDict[feature1]['dependent_features'].add(feature2)
						else:
							featureDict[feature1]['jointly_dependent_features'].add(feature2)

		dependencyOrder = []
		already_checked = set()
		for feature in cached_featureList:
			if feature not in already_checked:
				jointlyDependentFeatures = []
				for joint_feature in featureDict[feature]['jointly_dependent_features']:
					if joint_feature not in already_checked:
						jointlyDependentFeatures.append(joint_feature)
				jointlyDependentFeatures.append(feature)
				if debug:
					dependencyOrder.append(jointlyDependentFeatures)
				else:
					jointFeatures = []
					for feature_name in jointlyDependentFeatures:
						jointFeatures.append(featureDict[feature_name]['feature'])
					dependencyOrder.append(jointFeatures)

				for f in jointlyDependentFeatures: # to avoid double counting features
					already_checked.add(f)
		return dependencyOrder