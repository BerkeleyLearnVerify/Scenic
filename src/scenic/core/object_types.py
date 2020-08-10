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
from scenic.core.type_support import toVector, toHeading
from scenic.core.lazy_eval import needsLazyEvaluation
from scenic.core.utils import areEquivalent, cached_property
from scenic.core.errors import RuntimeParseError

## Abstract base class

class Constructible(Samplable):
	"""Abstract base class for Scenic objects.

	Scenic objects, which are constructed using specifiers, are implemented
	internally as instances of ordinary Python classes. This abstract class
	implements the procedure to resolve specifiers and determine values for
	the properties of an object, as well as several common methods supported
	by objects.
	"""

	def __init_subclass__(cls):
		# find all defaults provided by the class or its superclasses
		allDefs = collections.defaultdict(list)
		for sc in inspect.getmro(cls):
			if hasattr(sc, '__annotations__'):
				for prop, value in sc.__annotations__.items():
					allDefs[prop].append(PropertyDefault.forValue(value))

		# resolve conflicting defaults
		resolvedDefs = {}
		for prop, defs in allDefs.items():
			primary, rest = defs[0], defs[1:]
			spec = primary.resolveFor(prop, rest)
			resolvedDefs[prop] = spec
		cls.defaults = resolvedDefs

	@classmethod
	def withProperties(cls, props):
		assert all(reqProp in props for reqProp in cls.defaults)
		return cls(_internal=True, **props)

	def __init__(self, *args, _internal=False, **kwargs):
		if _internal:	# Object is being constructed internally; use fast path
			assert not args
			for prop, value in kwargs.items():
				assert not needsLazyEvaluation(value), (prop, value)
				object.__setattr__(self, prop, value)
			super().__init__(kwargs.values())
			self.properties = set(kwargs.keys())
			return

		# Validate specifiers
		name = type(self).__name__
		specifiers = list(args)
		for prop, val in kwargs.items():	# kwargs supported for internal use
			specifiers.append(Specifier(prop, val, internal=True))
		properties = dict()
		optionals = collections.defaultdict(list)
		defs = self.__class__.defaults
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

		# Normalize types of built-in properties
		self.properties = set(properties)
		if 'position' in properties:
			self.position = toVector(self.position, f'"position" of {self} not a vector')
		if 'heading' in properties:
			self.heading = toHeading(self.heading, f'"heading" of {self} not a heading')

		# Set up dependencies
		deps = []
		for prop in properties:
			assert hasattr(self, prop)
			val = getattr(self, prop)
			deps.append(val)
		super().__init__(deps)

		# Possibly register this object
		self._register()

	def _register(self):
		pass	# do nothing by default; may be overridden by subclasses

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
		if hasattr(self, 'properties') and 'name' in self.properties:
			return self.name
		else:
			return super().__repr__()

	def __repr__(self):
		if hasattr(self, 'properties'):
			allProps = { prop: getattr(self, prop) for prop in self.properties }
		else:
			allProps = '<under construction>'
		return f'{type(self).__name__}({allProps})'

## Mutators

class Mutator:
	"""An object controlling how the ``mutate`` statement affects an `Object`.

	A `Mutator` can be assigned to the ``mutator`` property of an `Object` to
	control the effect of the ``mutate`` statement. When mutation is enabled
	for such an object using that statement, the mutator's `appliedTo` method
	is called to compute a mutated version.
	"""

	def appliedTo(self, obj):
		"""Return a mutated copy of the object. Implemented by subclasses."""
		raise NotImplementedError

class PositionMutator(Mutator):
	"""Mutator adding Gaussian noise to ``position``. Used by `Point`.

	Attributes:
		stddev (float): standard deviation of noise
	"""
	def __init__(self, stddev):
		self.stddev = stddev

	def appliedTo(self, obj):
		noise = Vector(random.gauss(0, self.stddev), random.gauss(0, self.stddev))
		pos = pos + noise
		return (obj.copyWith(position=pos), True)		# allow further mutation

	def __eq__(self, other):
		if type(other) is not type(self):
			return NotImplemented
		return (other.stddev == self.stddev)

	def __hash__(self):
		return hash(self.stddev)

class HeadingMutator(Mutator):
	"""Mutator adding Gaussian noise to ``heading``. Used by `OrientedPoint`.

	Attributes:
		stddev (float): standard deviation of noise
	"""
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
	"""Implementation of the Scenic class ``Point``.

	The default mutator for `Point` adds Gaussian noise to ``position`` with
	a standard deviation given by the ``positionStdDev`` property.

	Attributes:
		position (`Vector`): Position of the point. Default value is the origin.
		visibleDistance (float): Distance for ``can see`` operator. Default value 50.
		width (float): Default value zero (only provided for compatibility with
		  operators that expect an `Object`).
		height (float): Default value zero.
	"""
	position: Vector(0, 0)
	width: 0
	height: 0
	visibleDistance: 50

	mutationEnabled: False
	mutator: PropertyDefault({'positionStdDev'}, {'additive'},
							 lambda self: PositionMutator(self.positionStdDev))
	positionStdDev: 1

	@cached_property
	def visibleRegion(self):
		return CircularRegion(self.position, self.visibleDistance)

	@cached_property
	def corners(self):
		return (self.position,)

	def toVector(self) -> Vector:
		return self.position

	def canSee(self, other) -> bool:	# TODO improve approximation?
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
	"""Implementation of the Scenic class ``OrientedPoint``.

	The default mutator for `OrientedPoint` adds Gaussian noise to ``heading``
	with a standard deviation given by the ``headingStdDev`` property, then
	applies the mutator for `Point`.

	Attributes:
		heading (float): Heading of the `OrientedPoint`. Default value 0 (North).
		viewAngle (float): View cone angle for ``can see`` operator. Default
		  value :math:`2\\pi`.
	"""
	heading: 0
	viewAngle: math.tau

	mutator: PropertyDefault({'headingStdDev'}, {'additive'},
		lambda self: HeadingMutator(self.headingStdDev))
	headingStdDev: math.radians(5)

	@cached_property
	def visibleRegion(self):
		return SectorRegion(self.position, self.visibleDistance,
		                    self.heading, self.viewAngle)

	def relativize(self, vec):
		pos = self.relativePosition(vec)
		return OrientedPoint(position=pos, heading=self.heading)

	def relativePosition(self, vec):
		return self.position.offsetRotated(self.heading, vec)

	def toHeading(self) -> float:
		return self.heading

## Object

class Object(OrientedPoint, RotatedRectangle):
	"""Implementation of the Scenic class ``Object``.

	Attributes:
		width (float): Width of the object, i.e. extent along its X axis.
		  Default value 1.
		height (float): Height of the object, i.e. extent along its Y axis.
		  Default value 1.
		allowCollisions (bool): Whether the object is allowed to intersect
		  other objects. Default value ``False``.
		requireVisible (bool): Whether the object is required to be visible
		  from the ``ego`` object. Default value ``True``.
		regionContainedIn (`Region` or ``None``): A `Region` the object is
		  required to be contained in. If ``None``, the object need only be
		  contained in the scenario's workspace.
		cameraOffset (`Vector`): Position of the camera for the ``can see``
		  operator, relative to the object's ``position``. Default ``0 @ 0``.
	"""
	width: 1
	height: 1
	allowCollisions: False
	requireVisible: True
	regionContainedIn: None
	cameraOffset: Vector(0, 0)
	behavior: None

	def __new__(cls, *args, **kwargs):
		obj = super().__new__(cls)
		# The _dynamicProxy attribute stores a mutable copy of the object used during
		# simulations, intercepting all attribute accesses to the original object;
		# we set this attribute very early to prevent problems during unpickling.
		object.__setattr__(obj, '_dynamicProxy', obj)
		return obj

	def __init__(self, *args, **kwargs):
		super().__init__(*args, **kwargs)
		self.hw = hw = self.width / 2
		self.hh = hh = self.height / 2
		self.radius = hypot(hw, hh)	# circumcircle; for collision detection
		self.inradius = min(hw, hh)	# incircle; for collision detection

		self._relations = []

	def _register(self):
		import scenic.syntax.veneer as veneer	# TODO improve?
		veneer.registerObject(self)

	def __getattribute__(self, name):
		proxy = object.__getattribute__(self, '_dynamicProxy')
		return object.__getattribute__(proxy, name)

	def __setattr__(self, name, value):
		proxy = object.__getattribute__(self, '_dynamicProxy')
		object.__setattr__(proxy, name, value)

	@cached_property
	def left(self):
		return self.relativize(Vector(-self.hw, 0))

	@cached_property
	def right(self):
		return self.relativize(Vector(self.hw, 0))

	@cached_property
	def front(self):
		return self.relativize(Vector(0, self.hh))

	@cached_property
	def back(self):
		return self.relativize(Vector(0, -self.hh))

	@cached_property
	def frontLeft(self):
		return self.relativize(Vector(-self.hw, self.hh))

	@cached_property
	def frontRight(self):
		return self.relativize(Vector(self.hw, self.hh))

	@cached_property
	def backLeft(self):
		return self.relativize(Vector(-self.hw, -self.hh))

	@cached_property
	def backRight(self):
		return self.relativize(Vector(self.hw, -self.hh))

	@cached_property
	def visibleRegion(self):
		camera = self.position.offsetRotated(self.heading, self.cameraOffset)
		return SectorRegion(camera, self.visibleDistance, self.heading, self.viewAngle)

	@cached_property
	def corners(self):
		hw, hh = self.hw, self.hh
		return (
			self.relativePosition(Vector(hw, hh)),
			self.relativePosition(Vector(-hw, hh)),
			self.relativePosition(Vector(-hw, -hh)),
			self.relativePosition(Vector(hw, -hh))
		)

	def show(self, workspace, plt, highlight=False):
		if needsSampling(self):
			raise RuntimeError('tried to show() symbolic Object')
		pos = self.position
		spos = workspace.scenicToSchematicCoords(pos)

		if highlight:
			# Circle around object
			rad = 1.5 * max(self.width, self.height)
			c = plt.Circle(spos, rad, color='g', fill=False)
			plt.gca().add_artist(c)
			# View cone
			ha = self.viewAngle / 2.0
			camera = self.position.offsetRotated(self.heading, self.cameraOffset)
			cpos = workspace.scenicToSchematicCoords(camera)
			for angle in (-ha, ha):
				p = camera.offsetRadially(20, self.heading + angle)
				edge = [cpos, workspace.scenicToSchematicCoords(p)]
				x, y = zip(*edge)
				plt.plot(x, y, 'b:')

		corners = [workspace.scenicToSchematicCoords(corner) for corner in self.corners]
		x, y = zip(*corners)
		color = self.color if hasattr(self, 'color') else (1, 0, 0)
		plt.fill(x, y, color=color)

		frontMid = averageVectors(corners[0], corners[1])
		baseTriangle = [frontMid, corners[2], corners[3]]
		triangle = [averageVectors(p, spos, weight=0.5) for p in baseTriangle]
		x, y = zip(*triangle)
		plt.fill(x, y, "w")
		plt.plot(x + (x[0],), y + (y[0],), color="k", linewidth=1)

def enableDynamicProxyFor(obj):
	object.__setattr__(obj, '_dynamicProxy', obj.copyWith())

def disableDynamicProxyFor(obj):
	object.__setattr__(obj, '_dynamicProxy', obj)
