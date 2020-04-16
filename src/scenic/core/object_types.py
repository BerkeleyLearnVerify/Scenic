"""Implementations of the built-in Scenic classes."""

import inspect
import collections
import math
import random

from scenic.core.distributions import Samplable, needsSampling
from scenic.core.specifiers import Specifier, PropertyDefault
from scenic.core.vectors import Vector
from scenic.core.geometry import RotatedRectangle, averageVectors, hypot, min, pointIsInCone
from scenic.core.regions import CircularRegion, SectorRegion
from scenic.core.type_support import toVector, toScalar
from scenic.core.lazy_eval import needsLazyEvaluation
from scenic.core.utils import areEquivalent, RuntimeParseError

## Abstract base class

class Constructible(Samplable):
	"""Abstract base class for Scenic objects.

	Scenic objects, which are constructed using specifiers, are implemented
	internally as instances of ordinary Python classes. This abstract class
	implements the procedure to resolve specifiers and determine values for
	the properties of an object, as well as several common methods supported
	by objects.
	"""

	@classmethod
	def defaults(cla):		# TODO improve so this need only be done once?
		# find all defaults provided by the class or its superclasses
		allDefs = collections.defaultdict(list)
		for sc in inspect.getmro(cla):
			if hasattr(sc, '__annotations__'):
				for prop, value in sc.__annotations__.items():
					allDefs[prop].append(PropertyDefault.forValue(value))

		# resolve conflicting defaults
		resolvedDefs = {}
		for prop, defs in allDefs.items():
			primary, rest = defs[0], defs[1:]
			spec = primary.resolveFor(prop, rest)
			resolvedDefs[prop] = spec
		return resolvedDefs

	@classmethod
	def withProperties(cls, props):
		assert all(reqProp in props for reqProp in cls.defaults())
		assert all(not needsLazyEvaluation(val) for val in props.values())
		specs = (Specifier(prop, val) for prop, val in props.items())
		return cls(*specs)

	def __init__(self, *args, **kwargs):
		# Validate specifiers
		name = type(self).__name__
		specifiers = list(args)
		for prop, val in kwargs.items():
			specifiers.append(Specifier(prop, val))
		properties = dict()
		optionals = collections.defaultdict(list)
		defs = self.defaults()
		for spec in specifiers:
			assert isinstance(spec, Specifier), (name, spec)
			prop = spec.property
			if prop in properties:
				raise RuntimeParseError(f'property "{prop}" of {name} specified twice')
			properties[prop] = spec
			for opt in spec.optionals:
				if opt in defs:		# do not apply optionals for properties this object lacks
					optionals[opt].append(spec)

		# Decide which optionals to use
		optionalsForSpec = collections.defaultdict(set)
		for opt, specs in optionals.items():
			if opt in properties:
				continue		# optionals do not override a primary specification
			if len(specs) > 1:
				raise RuntimeParseError(f'property "{opt}" of {name} specified twice (optionally)')
			assert len(specs) == 1
			spec = specs[0]
			properties[opt] = spec
			optionalsForSpec[spec].add(opt)

		# Add any default specifiers needed
		for prop in defs:
			if prop not in properties:
				spec = defs[prop]
				specifiers.append(spec)
				properties[prop] = spec

		# Topologically sort specifiers
		order = []
		seen, done = set(), set()

		def dfs(spec):
			if spec in done:
				return
			elif spec in seen:
				raise RuntimeParseError(f'specifier for property {spec.property} '
										'depends on itself')
			seen.add(spec)
			for dep in spec.requiredProperties:
				child = properties.get(dep)
				if child is None:
					raise RuntimeParseError(f'property {dep} required by '
											f'specifier {spec} is not specified')
				else:
					dfs(child)
			order.append(spec)
			done.add(spec)

		for spec in specifiers:
			dfs(spec)
		assert len(order) == len(specifiers)

		# Evaluate and apply specifiers
		for spec in order:
			spec.applyTo(self, optionalsForSpec[spec])

		# Set up dependencies
		deps = []
		for prop in properties:
			assert hasattr(self, prop)
			val = getattr(self, prop)
			if needsSampling(val):
				deps.append(val)
		super().__init__(deps)
		self.properties = set(properties)

	def sampleGiven(self, value):
		return self.withProperties({ prop: value[getattr(self, prop)]
								   for prop in self.properties })

	def allProperties(self):
		return { prop: getattr(self, prop) for prop in self.properties }

	def copyWith(self, **overrides):
		props = self.allProperties()
		props.update(overrides)
		return self.withProperties(props)

	def isEquivalentTo(self, other):
		if type(other) is not type(self):
			return False
		return areEquivalent(self.allProperties(), other.allProperties())

	def __str__(self):
		if hasattr(self, 'properties'):
			allProps = { prop: getattr(self, prop) for prop in self.properties }
		else:
			allProps = '<under construction>'
		return f'{type(self).__name__}({allProps})'

## Mutators

class Mutator:
	pass

class PositionMutator(Mutator):
	def __init__(self, stddev):
		self.stddev = stddev

	def appliedTo(self, obj):
		noise = Vector(random.gauss(0, self.stddev), random.gauss(0, self.stddev))
		pos = toVector(obj.position, '"position" not a vector')
		pos = pos + noise
		return (obj.copyWith(position=pos), True)		# allow further mutation

	def __eq__(self, other):
		if type(other) is not type(self):
			return NotImplemented
		return (other.stddev == self.stddev)

	def __hash__(self):
		return hash(self.stddev)

class HeadingMutator(Mutator):
	def __init__(self, stddev):
		self.stddev = stddev

	def appliedTo(self, obj):
		noise = random.gauss(0, self.stddev)
		h = obj.heading + noise
		return (obj.copyWith(heading=h), True)		# allow further mutation

	def __eq__(self, other):
		if type(other) is not type(self):
			return NotImplemented
		return (other.stddev == self.stddev)

	def __hash__(self):
		return hash(self.stddev)

## Point

class Point(Constructible):
	"""Implementation of the Scenic class ``Point``."""
	position: Vector(0, 0)
	width: 0
	height: 0
	visibleDistance: 50

	mutationEnabled: False
	mutator: PropertyDefault({'positionStdDev'}, {'additive'},
							 lambda self: PositionMutator(self.positionStdDev))
	positionStdDev: 1

	def __init__(self, *args, **kwargs):
		super().__init__(*args, **kwargs)
		self.position = toVector(self.position, f'"position" of {self} not a vector')
		self.corners = (self.position,)
		self.visibleRegion = CircularRegion(self.position, self.visibleDistance)

	def toVector(self):
		return self.position.toVector()

	def canSee(self, other):	# TODO improve approximation?
		for corner in other.corners:
			if self.visibleRegion.containsPoint(corner):
				return True
		return False

	def sampleGiven(self, value):
		sample = super().sampleGiven(value)
		if self.mutationEnabled:
			for mutator in self.mutator:
				if mutator is None:
					continue
				sample, proceed = mutator.appliedTo(sample)
				if not proceed:
					break
		return sample

	# Points automatically convert to Vectors when needed
	def __getattr__(self, attr):
		if hasattr(Vector, attr):
			return getattr(self.toVector(), attr)
		else:
			raise AttributeError(f"'{type(self).__name__}' object has no attribute '{attr}'")

## OrientedPoint

class OrientedPoint(Point):
	"""Implementation of the Scenic class ``OrientedPoint``."""
	heading: 0
	viewAngle: math.tau

	mutator: PropertyDefault({'headingStdDev'}, {'additive'},
		lambda self: HeadingMutator(self.headingStdDev))
	headingStdDev: math.radians(5)

	def __init__(self, *args, **kwargs):
		super().__init__(*args, **kwargs)
		self.heading = toScalar(self.heading, f'"heading" of {self} not a scalar')
		self.visibleRegion = SectorRegion(self.position, self.visibleDistance,
										  self.heading, self.viewAngle)

	def relativize(self, vec):
		pos = self.relativePosition(vec)
		return OrientedPoint(position=pos, heading=self.heading)

	def relativePosition(self, x, y=None):
		vec = x if y is None else Vector(x, y)
		pos = self.position.offsetRotated(self.heading, vec)
		return OrientedPoint(position=pos, heading=self.heading)

	def toHeading(self):
		return self.heading

## Object

class Object(OrientedPoint, RotatedRectangle):
	"""Implementation of the Scenic class ``Object``."""
	width: 1
	height: 1
	allowCollisions: False
	requireVisible: True
	regionContainedIn: None
	cameraOffset: Vector(0, 0)

	def __init__(self, *args, **kwargs):
		super().__init__(*args, **kwargs)
		import scenic.syntax.veneer as veneer	# TODO improve?
		veneer.registerObject(self)
		self.hw = hw = self.width / 2
		self.hh = hh = self.height / 2
		self.radius = hypot(hw, hh)	# circumcircle; for collision detection
		self.inradius = min(hw, hh)	# incircle; for collision detection
		self.left = self.relativePosition(-hw, 0)
		self.right = self.relativePosition(hw, 0)
		self.front = self.relativePosition(0, hh)
		self.back = self.relativePosition(0, -hh)
		self.frontLeft = self.relativePosition(-hw, hh)
		self.frontRight = self.relativePosition(hw, hh)
		self.backLeft = self.relativePosition(-hw, -hh)
		self.backRight = self.relativePosition(hw, -hh)
		self.corners = (self.frontRight.toVector(), self.frontLeft.toVector(),
			self.backLeft.toVector(), self.backRight.toVector())
		camera = self.position.offsetRotated(self.heading, self.cameraOffset)
		self.visibleRegion = SectorRegion(camera, self.visibleDistance,
										  self.heading, self.viewAngle)
		self._relations = []

	def show(self, workspace, plt, highlight=False):
		if needsSampling(self):
			raise RuntimeError('tried to show() symbolic Object')
		pos = self.position
		mpos = workspace.langToMapCoords(pos)

		if highlight:
			# Circle around object
			rad = 1.5 * max(self.width, self.height)
			c = plt.Circle(mpos, rad, color='g', fill=False)
			plt.gca().add_artist(c)
			# View cone
			ha = self.viewAngle / 2.0
			camera = self.position.offsetRotated(self.heading, self.cameraOffset)
			cpos = workspace.langToMapCoords(camera)
			for angle in (-ha, ha):
				p = camera.offsetRadially(20, self.heading + angle)
				edge = [cpos, workspace.langToMapCoords(p)]
				x, y = zip(*edge)
				plt.plot(x, y, 'b:')

		corners = [workspace.langToMapCoords(corner) for corner in self.corners]
		x, y = zip(*corners)
		color = self.color if hasattr(self, 'color') else (1, 0, 0)
		plt.fill(x, y, color=color)
		#plt.plot(x + (x[0],), y + (y[0],), color="g", linewidth=1)

		frontMid = averageVectors(corners[0], corners[1])
		baseTriangle = [frontMid, corners[2], corners[3]]
		triangle = [averageVectors(p, mpos, weight=0.5) for p in baseTriangle]
		x, y = zip(*triangle)
		plt.fill(x, y, "w")
		plt.plot(x + (x[0],), y + (y[0],), color="k", linewidth=1)
