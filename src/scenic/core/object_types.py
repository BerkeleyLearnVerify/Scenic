"""Implementations of the built-in Scenic classes."""

import collections
import math
import random

from scenic.core.distributions import Samplable, needsSampling
from scenic.core.specifiers import Specifier, PropertyDefault
from scenic.core.vectors import Vector
from scenic.core.geometry import (_RotatedRectangle, averageVectors, hypot, min,
                                  pointIsInCone)
from scenic.core.regions import CircularRegion, SectorRegion
from scenic.core.type_support import toVector, toHeading, toType
from scenic.core.lazy_eval import needsLazyEvaluation
from scenic.core.utils import DefaultIdentityDict, areEquivalent, cached_property
from scenic.core.errors import RuntimeParseError

## Abstract base class

class _Constructible(Samplable):
	"""Abstract base class for Scenic objects.

	Scenic objects, which are constructed using specifiers, are implemented
	internally as instances of ordinary Python classes. This abstract class
	implements the procedure to resolve specifiers and determine values for
	the properties of an object, as well as several common methods supported
	by objects.
	"""

	def __init_subclass__(cls):
		super().__init_subclass__()
		# find all defaults provided by the class or its superclasses
		allDefs = collections.defaultdict(list)
		for sc in cls.__mro__:
			if issubclass(sc, _Constructible) and hasattr(sc, '__annotations__'):
				for prop, value in sc.__annotations__.items():
					allDefs[prop].append(PropertyDefault.forValue(value))

		# resolve conflicting defaults and gather dynamic properties
		resolvedDefs = {}
		dyns = []
		for prop, defs in allDefs.items():
			primary, rest = defs[0], defs[1:]
			spec = primary.resolveFor(prop, rest)
			resolvedDefs[prop] = spec

			if any(defn.isDynamic for defn in defs):
				dyns.append(prop)
		cls._defaults = resolvedDefs
		cls._dynamicProperties = frozenset(dyns)

	@classmethod
	def withProperties(cls, props):
		assert all(reqProp in props for reqProp in cls._defaults)
		assert all(not needsLazyEvaluation(val) for val in props.values())
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

		# Resolve and apply specifiers
		specifiers = list(args)
		for prop, val in kwargs.items():	# kwargs supported for internal use
			specifiers.append(Specifier(prop, val))
		self._applySpecifiers(specifiers)

		# Set up dependencies
		deps = []
		for prop in self.properties:
			assert hasattr(self, prop)
			val = getattr(self, prop)
			deps.append(val)
		super().__init__(deps)

		# Possibly register this object
		self._register()

	def _applySpecifiers(self, specifiers, defs=None):
		# Validate specifiers
		name = self.__class__.__name__
		specifiers = list(specifiers)
		properties = dict()
		optionals = collections.defaultdict(list)
		if defs is None:
			defs = self.__class__._defaults
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
		for spec in specifiers:
			spec._dfs_state = 0

		def dfs(spec):
			if spec._dfs_state == 2:	# finished processing this specifier
				return
			elif spec._dfs_state == 1:	# specifier is being processed
				raise RuntimeParseError(f'specifier for property {spec.property} '
										'depends on itself')
			spec._dfs_state = 1
			for dep in spec.requiredProperties:
				child = properties.get(dep)
				if child is None:
					raise RuntimeParseError(f'property {dep} required by '
											f'specifier {spec} is not specified')
				else:
					dfs(child)
			order.append(spec)
			spec._dfs_state = 2

		for spec in specifiers:
			dfs(spec)
		assert len(order) == len(specifiers)
		for spec in specifiers:
			del spec._dfs_state

		# Evaluate and apply specifiers
		self.properties = set()		# will be filled by calls to _specify below
		self._evaluated = DefaultIdentityDict()		# temporary cache for lazily-evaluated values
		for spec in order:
			spec.applyTo(self, optionalsForSpec[spec])	# calls _specify
		del self._evaluated
		assert self.properties == set(properties)

	def _specify(self, prop, value):
		assert prop not in self.properties

		# Normalize types of some built-in properties
		if prop == 'position':
			value = toVector(value, f'"position" of {self} not a vector')
		elif prop == 'heading':
			value = toHeading(value, f'"heading" of {self} not a heading')

		self.properties.add(prop)
		object.__setattr__(self, prop, value)

	def _register(self):
		pass	# do nothing by default; may be overridden by subclasses

	def _override(self, specifiers):
		assert not needsSampling(self)
		oldVals = {}
		for spec in specifiers:
			prop = spec.property
			if prop in self._dynamicProperties:
				raise RuntimeParseError(f'cannot override dynamic property "{prop}"')
			if prop not in self.properties:
				raise RuntimeParseError(f'object has no property "{prop}" to override')
			oldVals[prop] = getattr(self, prop)
		defs = { prop: Specifier(prop, getattr(self, prop)) for prop in self.properties }
		self._applySpecifiers(specifiers, defs=defs)
		return oldVals

	def _revert(self, oldVals):
		for prop, val in oldVals.items():
			object.__setattr__(self, prop, val)

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
			return f'unnamed {self.__class__.__name__} ({id(self)})'

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
		pos = obj.position + noise
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

class Point(_Constructible):
	"""Implementation of the Scenic base class ``Point``.

	The default mutator for `Point` adds Gaussian noise to ``position`` with
	a standard deviation given by the ``positionStdDev`` property.

	Properties:
		position (`Vector`; dynamic): Position of the point. Default value is the origin.
		visibleDistance (float): Distance for ``can see`` operator. Default value 50.
		width (float): Default value zero (only provided for compatibility with
		  operators that expect an `Object`).
		length (float): Default value zero.

	.. note::

		If you're looking into Scenic's internals, note that `Point` is actually a
		subclass of the internal Python class `_Constructible`.
	"""
	position: PropertyDefault((), {'dynamic'}, lambda self: Vector(0, 0))
	width: 0
	length: 0
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

	Properties:
		heading (float; dynamic): Heading of the `OrientedPoint`. Default value 0
			(North).
		viewAngle (float): View cone angle for ``can see`` operator. Default
		  value :math:`2\\pi`.
	"""
	heading: PropertyDefault((), {'dynamic'}, lambda self: 0)
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

	def distancePast(self, vec):
		"""Distance past a given point, assuming we've been moving in a straight line."""
		diff = self.position - vec
		return diff.rotatedBy(-self.heading).y

	def toHeading(self) -> float:
		return self.heading

## Object

class Object(OrientedPoint, _RotatedRectangle):
	"""Implementation of the Scenic class ``Object``.

	This is the default base class for Scenic classes.

	Properties:
		width (float): Width of the object, i.e. extent along its X axis.
		  Default value 1.
		length (float): Length of the object, i.e. extent along its Y axis.
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

		speed (float; dynamic): Speed in dynamic simulations. Default value 0.
		velocity (`Vector`; *dynamic*): Velocity in dynamic simulations. Default value is
			the velocity determined by ``self.speed`` and ``self.heading``.
		angularSpeed (float; *dynamic*): Angular speed in dynamic simulations. Default
			value 0.

		behavior: Behavior for dynamic agents, if any (see :ref:`dynamics`). Default
			value ``None``.
	"""
	width: 1
	length: 1

	allowCollisions: False
	requireVisible: True
	regionContainedIn: None
	cameraOffset: Vector(0, 0)

	velocity: PropertyDefault(('speed', 'heading'), {'dynamic'},
	                          lambda self: Vector(0, self.speed).rotatedBy(self.heading))
	speed: PropertyDefault((), {'dynamic'}, lambda self: 0)
	angularSpeed: PropertyDefault((), {'dynamic'}, lambda self: 0)

	behavior: None
	lastActions: None

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
		self.hl = hl = self.length / 2
		self.radius = hypot(hw, hl)	# circumcircle; for collision detection
		self.inradius = min(hw, hl)	# incircle; for collision detection

		self._relations = []

	def _specify(self, prop, value):
		# Normalize types of some built-in properties
		if prop == 'behavior':
			import scenic.syntax.veneer as veneer	# TODO improve?
			value = toType(value, veneer.Behavior,
			               f'"behavior" of {self} not a behavior')
		super()._specify(prop, value)

	def _register(self):
		import scenic.syntax.veneer as veneer	# TODO improve?
		veneer.registerObject(self)

	def __getattribute__(self, name):
		proxy = object.__getattribute__(self, '_dynamicProxy')
		return object.__getattribute__(proxy, name)

	def __setattr__(self, name, value):
		proxy = object.__getattribute__(self, '_dynamicProxy')
		object.__setattr__(proxy, name, value)

	def __delattr__(self, name):
		proxy = object.__getattribute__(self, '_dynamicProxy')
		object.__delattr__(proxy, name)

	def startDynamicSimulation(self):
		"""Hook called at the beginning of each dynamic simulation.

		Does nothing by default; provided for objects to do simulator-specific
		initialization as needed.
		"""
		pass

	@cached_property
	def left(self):
		return self.relativize(Vector(-self.hw, 0))

	@cached_property
	def right(self):
		return self.relativize(Vector(self.hw, 0))

	@cached_property
	def front(self):
		return self.relativize(Vector(0, self.hl))

	@cached_property
	def back(self):
		return self.relativize(Vector(0, -self.hl))

	@cached_property
	def frontLeft(self):
		return self.relativize(Vector(-self.hw, self.hl))

	@cached_property
	def frontRight(self):
		return self.relativize(Vector(self.hw, self.hl))

	@cached_property
	def backLeft(self):
		return self.relativize(Vector(-self.hw, -self.hl))

	@cached_property
	def backRight(self):
		return self.relativize(Vector(self.hw, -self.hl))

	@cached_property
	def visibleRegion(self):
		camera = self.position.offsetRotated(self.heading, self.cameraOffset)
		return SectorRegion(camera, self.visibleDistance, self.heading, self.viewAngle)

	@cached_property
	def corners(self):
		hw, hl = self.hw, self.hl
		return (
			self.relativePosition(Vector(hw, hl)),
			self.relativePosition(Vector(-hw, hl)),
			self.relativePosition(Vector(-hw, -hl)),
			self.relativePosition(Vector(hw, -hl))
		)

	def show(self, workspace, plt, highlight=False):
		if needsSampling(self):
			raise RuntimeError('tried to show() symbolic Object')
		pos = self.position
		spos = workspace.scenicToSchematicCoords(pos)

		if highlight:
			# Circle around object
			rad = 1.5 * max(self.width, self.length)
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

def setDynamicProxyFor(obj, proxy):
	object.__setattr__(obj, '_dynamicProxy', proxy)

def disableDynamicProxyFor(obj):
	object.__setattr__(obj, '_dynamicProxy', obj)
