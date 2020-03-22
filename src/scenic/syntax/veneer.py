
"""Python implementations of Scenic language constructs.

This module is automatically imported by all Scenic programs. In addition to
defining the built-in functions, operators, specifiers, etc., it also stores
global state such as the list of all created Scenic objects.
"""

__all__ = (
	# Primitive statements and functions
	'ego', 'require', 'resample', 'param', 'mutate', 'verbosePrint',
	'sin', 'cos', 'hypot', 'max', 'min',
	# Prefix operators
	'Visible',
	'Front', 'Back', 'Left', 'Right',
	'FrontLeft', 'FrontRight', 'BackLeft', 'BackRight',
	# Infix operators
	'FieldAt', 'RelativeTo', 'OffsetAlong', 'RelativePosition',
	'RelativeHeading', 'ApparentHeading',
	'DistanceFrom', 'AngleTo', 'AngleFrom', 'Follow', 'CanSee',
	# Primitive types
	'Vector', 'VectorField', 'PolygonalVectorField',
	'Region', 'PointSetRegion', 'RectangularRegion', 'PolygonalRegion', 'PolylineRegion',
	'Workspace', 'Mutator',
	'Range', 'Options', 'Uniform', 'Normal',
	'VerifaiParameter', 'VerifaiRange', 'VerifaiDiscreteRange', 'VerifaiOptions',
	# Constructible types
	'Point', 'OrientedPoint', 'Object',
	# Specifiers
	'With',
	'At', 'In', 'Beyond', 'VisibleFrom', 'VisibleSpec', 'OffsetBy', 'OffsetAlongSpec',
	'Facing', 'FacingToward', 'ApparentlyFacing',
	'LeftSpec', 'RightSpec', 'Ahead', 'Behind',
	# Constants
	'everywhere', 'nowhere',
	# Temporary stuff... # TODO remove
	'PropertyDefault'
)

# various Python types and functions used in the language but defined elsewhere
from scenic.core.geometry import sin, cos, hypot, max, min
from scenic.core.vectors import Vector, VectorField, PolygonalVectorField
from scenic.core.regions import (Region, PointSetRegion, RectangularRegion,
	PolygonalRegion, PolylineRegion, everywhere, nowhere)
from scenic.core.workspaces import Workspace
from scenic.core.distributions import Range, Options, Normal
Uniform = lambda *opts: Options(opts)		# TODO separate these?
from scenic.core.external_params import (VerifaiParameter, VerifaiRange, VerifaiDiscreteRange,
                                         VerifaiOptions)
from scenic.core.object_types import Mutator, Point, OrientedPoint, Object
from scenic.core.specifiers import PropertyDefault	# TODO remove

# everything that should not be directly accessible from the language is imported here:
import inspect
from scenic.core.distributions import Distribution, toDistribution
from scenic.core.type_support import isA, toType, toTypes, toScalar, toHeading, toVector
from scenic.core.type_support import evaluateRequiringEqualTypes, underlyingType
from scenic.core.geometry import RotatedRectangle, normalizeAngle, apparentHeadingAtPoint
from scenic.core.object_types import Constructible
from scenic.core.specifiers import Specifier
from scenic.core.lazy_eval import DelayedArgument
from scenic.core.utils import RuntimeParseError
from scenic.core.external_params import ExternalParameter

### Internals

activity = 0
evaluatingRequirement = False
allObjects = []		# ordered for reproducibility
egoObject = None
globalParameters = {}
externalParameters = []		# ordered for reproducibility
pendingRequirements = {}
inheritedReqs = []		# TODO improve handling of these?

def isActive():
	"""Are we in the middle of compiling a Scenic module?

	The 'activity' global can be >1 when Scenic modules in turn import other
	Scenic modules."""
	return activity > 0

def activate():
	"""Activate the veneer when beginning to compile a Scenic module."""
	global activity
	activity += 1
	assert not evaluatingRequirement

def deactivate():
	"""Deactivate the veneer after compiling a Scenic module."""
	global activity, allObjects, egoObject, globalParameters, externalParameters
	global pendingRequirements, inheritedReqs
	activity -= 1
	assert activity >= 0
	assert not evaluatingRequirement
	allObjects = []
	egoObject = None
	globalParameters = {}
	externalParameters = []
	pendingRequirements = {}
	inheritedReqs = []

def registerObject(obj):
	"""Add a Scenic object to the global list of created objects.

	This is called by the Object constructor."""
	if activity > 0:
		assert not evaluatingRequirement
		assert isinstance(obj, Constructible)
		allObjects.append(obj)
	elif evaluatingRequirement:
		raise RuntimeParseError('tried to create an object inside a requirement')

def registerExternalParameter(value):
	"""Register a parameter whose value is given by an external sampler."""
	if activity > 0:
		assert isinstance(value, ExternalParameter)
		externalParameters.append(value)

### Primitive statements and functions

def ego(obj=None):
	"""Function implementing loads and stores to the 'ego' pseudo-variable.

	The translator calls this with no arguments for loads, and with the source
	value for stores.
	"""
	global egoObject
	if obj is None:
		if egoObject is None:
			raise RuntimeParseError('referred to ego object not yet assigned')
	elif not isinstance(obj, Object):
		raise RuntimeParseError('tried to make non-object the ego object')
	else:
		egoObject = obj
	return egoObject

def require(reqID, req, line, prob=1):
	"""Function implementing the require statement."""
	if evaluatingRequirement:
		raise RuntimeParseError('tried to create a requirement inside a requirement')
	# the translator wrapped the requirement in a lambda to prevent evaluation,
	# so we need to save the current values of all referenced names; throw in
	# the ego object too since it can be referred to implicitly
	assert reqID not in pendingRequirements
	pendingRequirements[reqID] = (req, getAllGlobals(req), egoObject, line, prob)

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

def resample(dist):
	"""The built-in resample function."""
	return dist.clone() if isinstance(dist, Distribution) else dist

def verbosePrint(msg):
	"""Built-in function printing a message when the verbosity is >0."""
	import scenic.syntax.translator as translator
	if translator.verbosity >= 1:
		indent = '  ' * activity if translator.verbosity >= 2 else '  '
		print(indent + msg)

def param(*quotedParams, **params):
	"""Function implementing the param statement."""
	if evaluatingRequirement:
		raise RuntimeParseError('tried to create a global parameter inside a requirement')
	for name, value in params.items():
		globalParameters[name] = toDistribution(value)
	assert len(quotedParams) % 2 == 0, quotedParams
	it = iter(quotedParams)
	for name, value in zip(it, it):
		globalParameters[name] = toDistribution(value)

def mutate(*objects):		# TODO update syntax
	"""Function implementing the mutate statement."""
	if evaluatingRequirement:
		raise RuntimeParseError('used mutate statement inside a requirement')
	if len(objects) == 0:
		objects = allObjects
	for obj in objects:
		if not isinstance(obj, Object):
			raise RuntimeParseError('"mutate X" with X not an object')
		obj.mutationEnabled = True

### Prefix operators

def Visible(region):
	"""The 'visible <region>' operator."""
	if not isinstance(region, Region):
		raise RuntimeParseError('"visible X" with X not a Region')
	return region.intersect(ego().visibleRegion)

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
	"""The 'distance from <vector> [to <vector>]' operator.

	If the 'to <vector>' is omitted, the position of ego is used.
	"""
	X = toVector(X, '"distance from X to Y" with X not a vector')
	if Y is None:
		Y = ego()
	Y = toVector(Y, '"distance from X to Y" with Y not a vector')
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
	elif isinstance(region, Options):
		return all(alwaysProvidesOrientation(opt) for opt in region.options)
	else:
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

	Specifies 'position', depending on 'height'. See other dependencies below.

	Allowed forms:

	* ``ahead of`` <oriented point> [``by`` <scalar/vector>] -- optionally specifies 'heading';
	* ``ahead of`` <vector> [``by`` <scalar/vector>] -- depends on 'heading'.

	If the 'by <scalar/vector>' is omitted, zero is used.
	"""
	return leftSpecHelper('ahead of', pos, dist, 'height', lambda dist: (0, dist),
	                      lambda self, dx, dy: Vector(dx, self.height / 2 + dy))

def Behind(pos, dist=0):
	"""The 'behind X [by Y]' polymorphic specifier.

	Specifies 'position', depending on 'height'. See other dependencies below.

	Allowed forms:
		behind <oriented point> [by <scalar/vector>] -- optionally specifies 'heading';
		behind <vector> [by <scalar/vector>] -- depends on 'heading'.

	If the 'by <scalar/vector>' is omitted, zero is used.
	"""
	return leftSpecHelper('behind', pos, dist, 'height', lambda dist: (0, dist),
	                      lambda self, dx, dy: Vector(dx, -self.height / 2 - dy))

def leftSpecHelper(syntax, pos, dist, axis, toComponents, makeOffset):
	extras = set()
	dType = underlyingType(dist)
	if dType is float or dType is int:
		dx, dy = toComponents(dist)
	elif dType is Vector:
		dx, dy = dist
	else:
		raise RuntimeParseError(f'"{syntax} X by D" with D not a number or vector')
	if isinstance(pos, OrientedPoint):		# TODO too strict?
		val = lambda self: pos.relativePosition(makeOffset(self, dx, dy))
		new = DelayedArgument({axis}, val)
		extras.add('heading')
	else:
		pos = toVector(pos, f'specifier "{syntax} X" with X not a vector')
		val = lambda self: pos.offsetRotated(self.heading, makeOffset(self, dx, dy))
		new = DelayedArgument({axis, 'heading'}, val)
	return Specifier('position', new, optionals=extras)
