
#### VENEER
#### implementations of language constructs

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
	# Constructible types
	'Point', 'OrientedPoint', 'Object',
	# Specifiers
	'With',
	'At', 'In', 'Beyond', 'VisibleFrom', 'VisibleSpec', 'OffsetBy', 'OffsetAlongSpec',
	'Facing', 'FacingToward', 'ApparentlyFacing',
	'LeftSpec', 'RightSpec', 'Ahead', 'Behind',
	# Temporary stuff... # TODO remove
	'PropertyDefault'
)

# various Python types and functions used in the language but defined elsewhere
from scenic.core.geometry import sin, cos, hypot, max, min
from scenic.core.vectors import Vector, VectorField, PolygonalVectorField
from scenic.core.regions import (Region, PointSetRegion, RectangularRegion,
	PolygonalRegion, PolylineRegion)
from scenic.core.workspaces import Workspace
from scenic.core.distributions import Range, Options, Normal
Uniform = lambda *opts: Options(opts)		# TODO separate these?
from scenic.core.object_types import Mutator, Point, OrientedPoint, Object
from scenic.core.specifiers import PropertyDefault	# TODO remove

# everything that should not be directly accessible from the language is imported here:
import inspect
from scenic.core.distributions import Distribution
from scenic.core.type_support import isA, toType, toTypes, toScalar, toHeading, toVector
from scenic.core.type_support import valueRequiringEqualTypes, underlyingType
from scenic.core.geometry import RotatedRectangle, normalizeAngle, apparentHeadingAtPoint
from scenic.core.specifiers import Specifier, DelayedArgument
from scenic.core.utils import RuntimeParseError

### Internals

activity = 0
allObjects = []		# ordered for reproducibility
egoObject = None
globalParameters = {}
pendingRequirements = {}
inheritedReqs = []		# TODO improve handling of these?

def isActive():
	return activity > 0

def activate():
	global activity
	activity += 1

def deactivate():
	global activity, allObjects, egoObject, globalParameters
	global pendingRequirements, inheritedReqs
	activity -= 1
	assert activity >= 0
	allObjects = []
	egoObject = None
	globalParameters = {}
	pendingRequirements = {}
	inheritedReqs = []

def registerObject(obj):
	if activity > 0:
		allObjects.append(obj)

### Primitive statements and functions

def ego(obj=None):
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
	# the translator wrapped the requirement in a lambda to prevent evaluation,
	# so we need to save the current values of all referenced names; throw in
	# the ego object too since it can be referred to implicitly
	assert reqID not in pendingRequirements
	pendingRequirements[reqID] = (req, getAllGlobals(req), egoObject, line, prob)

def getAllGlobals(req, restrictTo=None):
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
	return dist.clone() if isinstance(dist, Distribution) else dist

def verbosePrint(msg):
	import scenic.syntax.translator as translator
	if translator.verbosity >= 1:
		indent = '  ' * activity if translator.verbosity >= 2 else '  '
		print(indent + msg)

def param(**params):
	globalParameters.update(params)

def mutate(*objects):		# TODO update syntax
	if len(objects) == 0:
		objects = allObjects
	for obj in objects:
		if not isinstance(obj, Object):
			raise RuntimeParseError('"mutate X" with X not an object')
		obj.mutationEnabled = True

### Prefix operators

# visible <region>
def Visible(region):
	if not isinstance(region, Region):
		raise RuntimeParseError('"visible X" with X not a Region')
	return region.intersect(ego().visibleRegion)

# front of <object>, etc.
ops = (
	'front', 'back', 'left', 'right',
	'front left', 'front right',
	'back left', 'back right'
)
template = """\
def {function}(X):
	if not isinstance(X, Object):
		raise RuntimeParseError('"{syntax} of X" with X not an Object')
	return X.{property}
"""
for op in ops:
	func = ''.join(word.capitalize() for word in op.split(' '))
	prop = func[0].lower() + func[1:]
	definition = template.format(function=func, syntax=op, property=prop)
	exec(definition)

### Infix operators

# <field> at <vector>
def FieldAt(X, Y):
	if not isinstance(X, VectorField):
		raise RuntimeParseError('"X at Y" with X not a vector field')
	Y = toVector(Y, '"X at Y" with Y not a vector')
	return X[Y]

# F relative to G (with at least one of F, G a field, the other a field or heading)
# <vector> relative to <oriented point> (and vice versa)
# <vector> relative to <vector>
# <heading> relative to <heading>
def RelativeTo(X, Y):
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
			return valueRequiringEqualTypes(X + Y, X, Y, '"X relative to Y" with vector and scalar')

# <vector> offset along <heading> by <vector>
# <vector> offset along <field> by <vector>
def OffsetAlong(X, H, Y):
	X = toVector(X, '"X offset along H by Y" with X not a vector')
	Y = toVector(Y, '"X offset along H by Y" with Y not a vector')
	if isinstance(H, VectorField):
		H = H[X]
	H = toHeading(H, '"X offset along H by Y" with H not a heading or vector field')
	return X.offsetRotated(H, Y)

# relative position of <vector> from <vector>
def RelativePosition(X, Y=None):
	X = toVector(X, '"relative position of X from Y" with X not a vector')
	if Y is None:
		Y = ego()
	Y = toVector(Y, '"relative position of X from Y" with Y not a vector')
	return X - Y

# relative heading of <heading> [from <heading>]
def RelativeHeading(X, Y=None):
	X = toHeading(X, '"relative heading of X from Y" with X not a heading')
	if Y is None:
		Y = ego().heading
	else:
		Y = toHeading(Y, '"relative heading of X from Y" with Y not a heading')
	return normalizeAngle(X - Y)

# apparent heading of <oriented point> from <vector>
def ApparentHeading(X, Y=None):
	if not isinstance(X, OrientedPoint):
		raise RuntimeParseError('"apparent heading of X from Y" with X not an OrientedPoint')
	if Y is None:
		Y = ego()
	Y = toVector(Y, '"relative heading of X from Y" with Y not a vector')
	return apparentHeadingAtPoint(X.position, X.heading, Y)

# distance from <vector> to <vector>
def DistanceFrom(X, Y=None):
	X = toVector(X, '"distance from X to Y" with X not a vector')
	if Y is None:
		Y = ego()
	Y = toVector(Y, '"distance from X to Y" with Y not a vector')
	return X.distanceTo(Y)

# angle to <vector>
def AngleTo(X):
	X = toVector(X, '"angle to X" with X not a vector')
	return ego().angleTo(X)

# angle from <vector> to <vector>
def AngleFrom(X, Y):
	X = toVector(X, '"angle from X to Y" with X not a vector')
	Y = toVector(Y, '"angle from X to Y" with Y not a vector')
	return X.angleTo(Y)

# follow <field> from <vector> for <number>
def Follow(F, X, D):
	if not isinstance(F, VectorField):
		raise RuntimeParseError('"follow F from X for D" with F not a vector field')
	X = toVector(X, '"follow F from X for D" with X not a vector')
	D = toScalar(D, '"follow F from X for D" with D not a number')
	pos = F.followFrom(X, D)
	heading = F[pos]
	return OrientedPoint(position=pos, heading=heading)

# <point> can see <vector>
# <point> can see <object>
def CanSee(X, Y):
	if not isinstance(X, Point):
		raise RuntimeParseError('"X can see Y" with X not a Point')
	if isinstance(Y, Object):
		return X.canSee(Y)
	else:
		Y = toVector(Y, '"X can see Y" with Y not a vector')
		return X.visibleRegion.containsPoint(Y)

### Specifiers

# with <property> <value>
def With(prop, val):
	return Specifier(prop, val)

# at <vector>
def At(pos):
	pos = toVector(pos, 'specifier "at X" with X not a vector')
	return Specifier('position', pos)

# in/on <region>
def In(region):
	# TODO fix type checking
	#region = toType(region, Region, 'specifier "in/on R" with R not a Region')
	if isinstance(region, Distribution):
		extras = {'heading'}	# TODO fix hack!!!
	else:
		extras = {} if region.orientation is None else {'heading'}
	return Specifier('position', Region.uniformPointIn(region), optionals=extras)

# beyond <vector> by <scalar-or-vector> from <vector>
def Beyond(pos, offset, fromPt=None):
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

# visible from <Point>
# visible from <OrientedPoint>
def VisibleFrom(base):
	if not isinstance(base, Point):
		raise RuntimeParseError('specifier "visible from O" with O not a Point')
	return Specifier('position', Region.uniformPointIn(base.visibleRegion))

# visible
def VisibleSpec():
	return VisibleFrom(ego())

# offset by <vector>
def OffsetBy(offset):
	offset = toVector(offset, 'specifier "offset by X" with X not a vector')
	pos = RelativeTo(offset, ego()).toVector()
	return Specifier('position', pos)

# offset along <heading> by <vector>
# offset along <field> by <vector>
def OffsetAlongSpec(direction, offset):
	return Specifier('position', OffsetAlong(ego(), direction, offset))

# facing <field>
# facing <number>
def Facing(heading):
	if isinstance(heading, VectorField):
		return Specifier('heading', DelayedArgument({'position'},
		                                            lambda self: heading[self.position]))
	else:
		heading = toHeading(heading, 'specifier "facing X" with X not a heading or vector field')
		return Specifier('heading', heading)

# facing toward <vector>
def FacingToward(pos):
	pos = toVector(pos, 'specifier "facing toward X" with X not a vector')
	return Specifier('heading', DelayedArgument({'position'},
	                                            lambda self: self.position.angleTo(pos)))

# apparently facing <number> from <vector>
def ApparentlyFacing(heading, fromPt=None):
	heading = toHeading(heading, 'specifier "apparently facing X" with X not a heading')
	if fromPt is None:
		fromPt = ego()
	fromPt = toVector(fromPt, 'specifier "apparently facing X from Y" with Y not a vector')
	value = lambda self: fromPt.angleTo(self.position) + heading
	return Specifier('heading', DelayedArgument({'position'}, value))

# left of <oriented point> [by <scalar>]
# left of <vector> [by <scalar>]
def LeftSpec(pos, dist=0):
	return leftSpecHelper('left of', pos, dist, 'width', lambda dist: (dist, 0),
	                      lambda self, dx, dy: Vector(-self.width / 2 - dx, dy))

# right of <oriented point>
# right of <vector>
def RightSpec(pos, dist=0):
	return leftSpecHelper('right of', pos, dist, 'width', lambda dist: (dist, 0),
	                      lambda self, dx, dy: Vector(self.width / 2 + dx, dy))

# ahead of <oriented point> [by <scalar-or-vector>]
# ahead of <vector> [by <scalar-or-vector>]
def Ahead(pos, dist=0):
	return leftSpecHelper('ahead of', pos, dist, 'height', lambda dist: (0, dist),
	                      lambda self, dx, dy: Vector(dx, self.height / 2 + dy))

# behind <oriented point> [by <scalar-or-vector>]
# behind <vector> [by <scalar-or-vector>]
def Behind(pos, dist=0):
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
