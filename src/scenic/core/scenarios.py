
import random
import time

from scenic.core.distributions import Samplable, RejectionException, needsSampling
from scenic.core.lazy_eval import needsLazyEvaluation
from scenic.core.workspaces import Workspace
from scenic.core.vectors import Vector
from scenic.core.utils import InvalidScenarioError
from scenic.syntax.veneer import isABehavior, RequirementType, BoundRequirement

class Scene:
	"""A scene generated from a Scenic scenario"""
	def __init__(self, workspace, simulator, objects, egoObject, params,
	             alwaysReqs=tuple(), terminationConds=tuple()):
		self.workspace = workspace
		self.simulator = simulator
		self.objects = tuple(objects)
		self.egoObject = egoObject
		self.params = params
		self.alwaysRequirements = tuple(alwaysReqs)
		self.terminationConditions = tuple(terminationConds)

	def show(self, zoom=None, block=True):
		"""Render a schematic of the scene for debugging"""
		import matplotlib.pyplot as plt
		# display map
		self.workspace.show(plt)
		# draw objects
		for obj in self.objects:
			obj.show(self.workspace, plt, highlight=(obj is self.egoObject))
		# zoom in if requested
		if zoom != None:
			self.workspace.zoomAround(plt, self.objects, expansion=zoom)
		plt.show(block=block)

	def simulate(self, maxSteps=None, maxIterations=100, verbosity=0):
		"""Run a simulation of this scene."""
		if self.simulator is None:
			raise RuntimeError('tried to simulate Scene which does not have a Simulator')
		return self.simulator.simulate(self, maxSteps=maxSteps, maxIterations=maxIterations,
		                               verbosity=verbosity)

class Scenario:
	"""A Scenic scenario"""
	def __init__(self, workspace, simulator,
	             objects, egoObject,
	             params,
	             requirements, requirementDeps):
		if workspace is None:
			workspace = Workspace()		# default empty workspace
		self.workspace = workspace
		self.simulator = simulator		# simulator for dynamic scenarios
		ordered = []
		for obj in objects:
			ordered.append(obj)
			if obj is egoObject:	# make ego the first object
				ordered[0], ordered[-1] = ordered[-1], ordered[0]
		assert ordered[0] is egoObject
		self.objects = tuple(ordered)
		self.egoObject = egoObject
		self.params = dict(params)
		staticReqs, alwaysReqs, terminationConds = [], [], []
		for req in requirements:
			if req.ty is RequirementType.require:
				place = staticReqs
			elif req.ty is RequirementType.requireAlways:
				place = alwaysReqs
			elif req.ty is RequirementType.terminateWhen:
				place = terminationConds
			else:
				raise RuntimeError(f'requirement {req} has unknown type!')
			place.append(req)
		self.requirements, self.alwaysRequirements = tuple(staticReqs), tuple(alwaysReqs)
		self.initialRequirements = self.requirements + self.alwaysRequirements
		assert all(req.constrainsSampling for req in self.initialRequirements)
		self.terminationConditions = tuple(terminationConds)
		# dependencies must use fixed order for reproducibility
		paramDeps = tuple(p for p in self.params.values() if isinstance(p, Samplable))
		self.dependencies = self.objects + paramDeps + tuple(requirementDeps)
		self.validate()

	def containerOfObject(self, obj):
		if hasattr(obj, 'regionContainedIn') and obj.regionContainedIn is not None:
			return obj.regionContainedIn
		else:
			return self.workspace.region

	def validate(self):
		"""Make some simple static checks for inconsistent built-in requirements."""
		objects = self.objects
		staticVisibility = not needsSampling(self.egoObject.visibleRegion)
		staticBounds = [self.hasStaticBounds(obj) for obj in objects]
		for i in range(len(objects)):
			oi = objects[i]
			# skip objects with unknown positions or bounding boxes
			if not staticBounds[i]:
				continue
			# Require object to be contained in the workspace/valid region
			container = self.containerOfObject(oi)
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

	def generate(self, maxIterations=2000, verbosity=0):
		objects = self.objects

		# choose which custom requirements will be enforced for this sample
		activeReqs = [req for req in self.initialRequirements if random.random() <= req.prob]

		# do rejection sampling until requirements are satisfied
		rejection = True
		iterations = 0
		while rejection is not None:
			if verbosity >= 2 and iterations > 0:
				print(f'  Rejected sample {iterations} because of: {rejection}')
			if iterations >= maxIterations:
				raise RuntimeError(f'failed to generate scenario in {iterations} iterations')
			iterations += 1
			try:
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
				assert isinstance(sampledObj.position, Vector)
				sampledObj.heading = float(sampledObj.heading)
				behavior = sampledObj.behavior
				if behavior is not None and not isABehavior(behavior):
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
			sampledValue = sample[value] if isinstance(value, Samplable) else value
			assert not needsLazyEvaluation(sampledValue)
			sampledParams[param] = sampledValue
		alwaysReqs = (BoundRequirement(req, sample) for req in self.alwaysRequirements)
		terminationConds = (BoundRequirement(req, sample) for req in self.terminationConditions)
		scene = Scene(self.workspace, self.simulator, sampledObjects, ego, sampledParams,
		              alwaysReqs, terminationConds)
		return scene, iterations
