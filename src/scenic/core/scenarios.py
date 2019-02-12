
import random
import time

from scenic.core.distributions import Samplable, RejectionException, needsSampling
from scenic.core.workspaces import Workspace
from scenic.core.vectors import Vector

class Scene:
	"""A scene generated from a Scenic scenario"""
	def __init__(self, workspace, objects, egoObject, params):
		self.workspace = workspace
		self.objects = tuple(objects)
		self.egoObject = egoObject
		self.params = params

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

class Scenario:
	"""A Scenic scenario"""
	def __init__(self, workspace,
	             objects, egoObject,
	             params,
	             requirements, requirementDeps):
		if workspace is None:
			workspace = Workspace()		# default empty workspace
		self.workspace = workspace
		ordered = []
		for obj in objects:
			ordered.append(obj)
			if obj is egoObject:	# make ego the first object
				ordered[0], ordered[-1] = ordered[-1], ordered[0]
		assert ordered[0] is egoObject
		self.objects = tuple(ordered)
		self.egoObject = egoObject
		self.params = dict(params)
		self.requirements = tuple(requirements)
		# dependencies must use fixed order for reproducibility
		self.dependencies = self.objects + tuple(self.params.values()) + tuple(requirementDeps)

	def containerOfObject(self, obj):
		if hasattr(obj, 'regionContainedIn') and obj.regionContainedIn is not None:
			return obj.regionContainedIn
		else:
			return self.workspace.region

	def generate(self, maxIterations=2000, verbosity=0):
		objects = self.objects

		# choose which custom requirements will be enforced for this sample
		activeReqs = [req for req, prob in self.requirements if random.random() <= prob]

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
				if not req(sample):
					rejection = 'user-specified requirement'
					break

		# obtained a valid sample; assemble a scene from it
		sampledObjects = tuple(sample[obj] for obj in objects)
		sampledParams = { param: sample[value] for param, value in self.params.items() }
		scene = Scene(self.workspace, sampledObjects, ego, sampledParams)
		return scene, iterations
