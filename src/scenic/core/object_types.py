"""Implementations of the built-in Scenic classes.

Defines the 3 Scenic classes `Point`, `OrientedPoint`, and `Object`, and associated
helper code (notably their base class `Constructible`, which implements the handling of
property definitions and :ref:`specifier resolution`).

.. warning::

    In 2D compatibility mode, these classes are overwritten with 2D analogs. While
    we make an effort to map imports to the correct class, this only works if imports
    use the form ``import scenic.core.object_types as object_types`` followed by accessing
    ``object_types.Object``. If you instead use ``from scenic.core.object_types import Object``,
    you may get the wrong class.

"""

import typing
import warnings
import collections
import math
import random
import numpy as np
import shapely
import trimesh
from abc import ABC, abstractmethod
from functools import lru_cache

from scenic.core.distributions import (Samplable, RandomControlFlowError, MultiplexerDistribution,
                                       needsSampling, distributionMethod, distributionFunction,
                                       supportInterval, toDistribution)
from scenic.core.specifiers import Specifier, PropertyDefault, ModifyingSpecifier
from scenic.core.vectors import Vector, Orientation, alwaysGlobalOrientation
from scenic.core.geometry import (averageVectors, hypot, min,
                                  pointIsInCone, normalizeAngle)
from scenic.core.regions import (Region, CircularRegion, SectorRegion, MeshVolumeRegion, MeshSurfaceRegion, 
                                  BoxRegion, SpheroidRegion, DefaultViewRegion, EmptyRegion, PolygonalRegion,
                                  convertToFootprint)
from scenic.core.type_support import toVector, toHeading, toType, toScalar, toOrientation, underlyingType
from scenic.core.lazy_eval import LazilyEvaluable, isLazy, needsLazyEvaluation
from scenic.core.serialization import dumpAsScenicCode
from scenic.core.utils import DefaultIdentityDict, cached_property
from scenic.core.errors import InvalidScenarioError, SpecifierError
from scenic.core.shapes import Shape, BoxShape, MeshShape
from scenic.core.regions import IntersectionRegion

## Types
#: Type alias for an interval (a pair of floats).
Interval = typing.Tuple[float, float]
#: Type alias for limits on dimensions (a triple of intervals).
DimensionLimits = typing.Tuple[Interval, Interval, Interval]

## Abstract base class

class Constructible(Samplable):
    """Abstract base class for Scenic objects.

    Scenic objects, which are constructed using specifiers, are implemented
    internally as instances of ordinary Python classes. This abstract class
    implements the procedure to resolve specifiers and determine values for
    the properties of an object, as well as several common methods supported
    by objects.

    .. warning::

        This class is an implementation detail, and none of its methods should be
        called directly from a Scenic program.
    """

    _dynamicProperties = {}

    def __init_subclass__(cls):
        super().__init_subclass__()
        # Find all defaults provided by the class or its superclasses
        allDefs = collections.defaultdict(list)

        for sc in cls.__mro__:
            if issubclass(sc, Constructible) and hasattr(sc, '_scenic_properties'):
                for prop, value in sc._scenic_properties.items():
                    allDefs[prop].append(PropertyDefault.forValue(value))

        # Resolve conflicting defaults and gather dynamic properties
        resolvedDefs = {}
        dyns = []
        finals = []
        for prop, defs in allDefs.items():
            primary, rest = defs[0], defs[1:]
            spec = primary.resolveFor(prop, rest)
            resolvedDefs[prop] = spec

            if any(defn.isDynamic for defn in defs):
                dyns.append(prop)
            if primary.isFinal:
                finals.append(prop)
        cls._defaults = resolvedDefs
        cls._finalProperties = tuple(finals)

        # Determine types of dynamic properties
        dynTypes = {}
        inst = None
        for prop in dyns:
            ty = super(cls, cls)._dynamicProperties.get(prop)
            if not ty:
                # First time this property has been defined; create a dummy object to
                # run specifier resolution and determine the property's default value
                if not inst:
                    inst = cls._withSpecifiers((), register=False)
                ty = underlyingType(getattr(inst, prop))
            dynTypes[prop] = ty
        cls._dynamicProperties = dynTypes

    def __new__(cls, *args, _internal=False, **kwargs):
        if not _internal:
            # Catch users trying to instantiate a Scenic class like a Python class
            raise InvalidScenarioError('Scenic classes must be instantiated with "new"')
        return super().__new__(cls)

    def __getnewargs_ex__(self):
        return ((), dict(_internal=True))

    def __init__(self, properties, constProps=frozenset(), _internal=False):
        for prop, value in properties.items():
            assert not needsLazyEvaluation(value), (prop, value)
            object.__setattr__(self, prop, value)
        super().__init__(properties.values())
        self.properties = tuple(sorted(properties.keys()))
        self._propertiesSet = set(self.properties)
        self._constProps = constProps

    @classmethod
    def _withProperties(cls, properties, constProps=None):
        """Create an instance with the given property values.

        Values of unspecified properties are determined by specifier resolution
        as usual.
        """
        specs = []
        for prop, val in properties.items():
            specs.append(Specifier("<internal>", {prop: 1}, {prop: val}))
        return cls._withSpecifiers(specs, constProps=constProps, register=False)

    @classmethod
    def _with(cls, **properties):
        # Shorthand form of _withProperties
        return cls._withProperties(properties)

    @classmethod
    def _withSpecifiers(cls, specifiers, constProps=None, register=True):
        """Create an instance from the given specifiers."""
        # Resolve specifiers
        newspecs = cls._prepareSpecifiers(specifiers)
        properties, consts = cls._resolveSpecifiers(newspecs)
        if constProps is None:
            constProps = consts

        # Catch properties which would conflict with ordinary attributes
        for prop in properties:
            if hasattr(cls, prop):
                raise SpecifierError(
                    f"Property {prop} would overwrite an attribute with the same name."
                )

        # Create the object
        obj = cls(properties, constProps=constProps, _internal=True)

        # Possibly register this object
        if register:
            obj._register()

        return obj

    @classmethod
    def _prepareSpecifiers(cls, specifiers):
        # This is a hook for subclasses to modify the specifier list.
        return specifiers

    @classmethod
    def _resolveSpecifiers(cls, specifiers, defaults=None, overriding=False):
        specifiers = list(specifiers)

        # Declare properties dictionary which maps properties to the specifier
        # that will specify that property.
        properties = dict()

        # Declare modifying dictionary, which maps properties to a specifier
        # that will modify that property.
        modifying = dict()

        # Dictionary mapping properties set so far to the priority with which they have
        # been set.
        priorities = dict()

        # Extract default property values dictionary and set of final properties,
        # unless defaults is overriden.
        if defaults is None:
            defaults = cls._defaults

        finals = cls._finalProperties

        # Check for incompatible specifier combinations
        specifier_names = [spec.name for spec in specifiers]

        if "On" in specifier_names:
            if collections.Counter(specifier_names)["On"] > 1:
                raise SpecifierError(f'Cannot use "On" specifier to modify "On" specifier')

        # Split the specifiers into two groups, normal and modifying. Normal specifiers set all relevant properties
        # first. Then modifying specifiers can modify or set additional properties
        normal_specifiers = [spec for spec in specifiers if not isinstance(spec, ModifyingSpecifier)]
        modifying_specifiers = [spec for spec in specifiers if isinstance(spec, ModifyingSpecifier)]

        
        # For each property specified by a normal specifier:
        #   - If not in properties specified, properties[p] = specifier
        #   - Otherwise, if property specified, check if specifier's priority is higher. If so, replace it with specifier

        #Priorties are inversed: A lower priority number means semantically that it has a higher priority level

        for spec in normal_specifiers:
            assert isinstance(spec, Specifier), (name, spec)

            # Iterate over each property.
            for prop in spec.priorities:
                # Check if this is a final property has been specified. If so, throw an assertion or error,
                # depending on whether or not this object is internal.
                if prop in finals:
                    raise SpecifierError(f'property "{prop}" cannot be directly specified')


                if prop in properties:
                    # This property already exists. Check that it has not already been specified
                    # at equal priority level. Then if it was previously specified at a lower priority
                    # level, override it with the value that this specifier sets.
                    if spec.priorities[prop] == priorities[prop]:
                        raise SpecifierError(f'property "{prop}" specified twice with the same priority')
                    if spec.priorities[prop] < priorities[prop]:
                        properties[prop] = spec
                        priorities[prop] = spec.priorities[prop]
                else:
                    # This property has not already been specified, so we should initialize it.
                    properties[prop] = spec
                    priorities[prop] = spec.priorities[prop]


        # If a modifying specifier specifies the property with a higher priority,
        # set the object's property to be specified by the modifying specifier. Otherwise,
        # if the property exists and the priorities match, object needs to be specified
        # by the original specifier then the resulting value is modified by the
        # modifying specifier. 

        # If the property is not yet being specified, the modifying specifier will 
        # act as a normal specifier for that property. 

        deprecate = []
        for spec in modifying_specifiers:
            for prop in spec.priorities:
                # If it has already been modified, which also implies this property has already been specified.


                # Now we check if the propert has already been specified
                if prop in properties:
                    # This property has already been specified, so we should either modify
                    # it or specify it.

                    if spec.priorities[prop] < priorities[prop]:
                        # Higher priority level, so it specifies
                        properties[prop] = spec
                        priorities[prop] = spec.priorities[prop]
                        deprecate.append(prop)
                    elif prop in spec.modifiable_props:
                        # This specifer can modify this prop, so we set it to do so after
                        # first checking it has not already been modified.
                        if prop in modifying:
                            raise SpecifierError(f'property "{prop}" of {name} modified twice.')

                        modifying[prop] = spec
                else:
                    # This property has not been specified, so we should specify it.
                    properties[prop] = spec
                    priorities[prop] = spec.priorities[prop]
                    deprecate.append(prop)

        # Add any default specifiers needed
        _defaultedProperties = set()
        for prop, default_spec in defaults.items():
            if prop not in priorities:
                specifiers.append(default_spec)
                properties[prop] = default_spec
                _defaultedProperties.add(prop)

        # Create the actual_props dictionary, which maps each specifier to a set of properties
        # it is actually specifying or modifying.
        actual_props = {spec: [] for spec in specifiers}
        for prop in properties:
            # Extract the specifier that is specifying this prop and add it to the
            # specifier's entry in actual_props
            specifying_spec = properties[prop]
            actual_props[specifying_spec].append(prop)

            # If a specifier modifies this property, add this prop to the specifiers
            # actual_props list.
            if prop in modifying:
                modifying_spec = modifying[prop]
                actual_props[modifying_spec].append(prop)

        # Create an inversed modifying dictionary that specifiers to the properties they
        # are modifying.
        modifying_inv = {spec:prop for prop, spec in modifying.items()}

        # Topologically sort specifiers. Specifiers become vertices and the properties
        # those specifiers depend on become the in-edges of each vertex. The specifiers
        # are then sorted topologically according to this graph.
        order = []
        for spec in specifiers:
            spec._dfs_state = 0

        def dfs(spec):
            if spec._dfs_state == 2:    # finished processing this specifier
                return
            elif spec._dfs_state == 1:  # specifier is being processed
                raise SpecifierError(f'specifier {spec.name} depends on itself')
            spec._dfs_state = 1

            # Recurse on dependencies
            for dep in spec.requiredProperties:
                child = modifying.get(dep)
                if not child:
                    child = properties.get(dep)

                if child is None:
                    raise SpecifierError(f'property {dep} required by '
                                         f'specifier {spec} is not specified')
                else:
                    dfs(child)

            # If this is a modifying specifier, recurse on the specifier
            # that specifies the property being modified.
            if spec in modifying_inv:
                specifying_spec = properties[modifying_inv[spec]]
                dfs(specifying_spec)

            order.append(spec)
            spec._dfs_state = 2

        for spec in specifiers:
            dfs(spec)
        assert len(order) == len(specifiers)
        for spec in specifiers:
            del spec._dfs_state

        context = LazilyEvaluable.makeContext()
        for spec in order:
            specifiedValues = spec.getValuesFor(context)
            for prop in actual_props[spec]:
                assert not hasattr(context, prop) or prop in modifying, (prop, spec)
                value = toDistribution(specifiedValues[prop])
                cls._specify(context, prop, value)
        properties = LazilyEvaluable.getContextValues(context)

        constProps = frozenset({
            prop for prop in _defaultedProperties
            if not needsSampling(properties[prop])
        })
        return properties, constProps

    @classmethod
    def _specify(cls, context, prop, value):
        # Normalize types of some built-in properties
        if prop in ('position', 'velocity', 'cameraOffset', 'positionStdDev', 'orientationStdDev'):
            value = toVector(value, f'"{prop}" of {cls.__name__} not a vector')
        elif prop in ('width', 'length', 'visibleDistance',
                      'viewAngle', 'speed', 'angularSpeed',
                      'yaw', 'pitch', 'roll'):
            value = toScalar(value, f'"{prop}" of {cls.__name__} not a scalar')

        if prop in ['yaw', 'pitch', 'roll']:
            value = normalizeAngle(value)

        if prop == "parentOrientation":
            value = toOrientation(value)

        if prop == 'regionContainedIn':
            # 2D regions can't contain objects, so we automatically use their footprint.
            value = convertToFootprint(value)

        object.__setattr__(context, prop, value)

    def _register(self):
        import scenic.syntax.veneer as veneer   # TODO improve?
        veneer.registerInstance(self)

    def _override(self, specifiers):
        assert not needsSampling(self)
        # Validate properties being overridden and gather their old values
        oldVals = {}
        for spec in specifiers:
            for prop in spec.priorities:
                if prop in self._dynamicProperties:
                    raise SpecifierError(f'cannot override dynamic property "{prop}"')
                if prop not in self._propertiesSet:
                    raise SpecifierError(f'object has no property "{prop}" to override')
                oldVals[prop] = getattr(self, prop)

        # Perform specifier resolution to find the new values of all properties
        defs = {
            prop: Specifier("OverrideDefault", {prop: -1}, {prop: getattr(self, prop)})
            for prop in self.properties
        }
        newprops, _ = self._resolveSpecifiers(specifiers, defaults=defs)

        # Apply the new values
        for prop, val in newprops.items():
            object.__setattr__(self, prop, val)

        return oldVals

    def _revert(self, oldVals):
        for prop, val in oldVals.items():
            object.__setattr__(self, prop, val)

    def sampleGiven(self, value):
        if not needsSampling(self):
            return self
        props = { prop: value[getattr(self, prop)] for prop in self.properties }
        return type(self)(props, constProps=self._constProps, _internal=True)

    def _allProperties(self):
        return { prop: getattr(self, prop) for prop in self.properties }

    def _copyWith(self, **overrides):
        """Copy this object, possibly overriding some of its properties."""
        # Copy all properties except for dynamic final values, which will retain their default values
        props = {
            prop: val
            for prop, val in self._allProperties().items()
            if prop not in set(self._finalProperties)
        }
        props.update(overrides)
        constProps = self._constProps.difference(overrides)
        return self._withProperties(props, constProps=constProps)

    def dumpAsScenicCode(self, stream, skipConstProperties=True):
        stream.write(f"new {self.__class__.__name__}")
        first = True
        for prop in self.properties:
            if skipConstProperties and prop in self._constProps:
                continue
            if prop in self._finalProperties:
                continue
            if prop == 'position':
                spec = 'at'
            else:
                spec = f'with {prop}'
            if first:
                stream.write(' ')
                first = False
            else:
                stream.write(',\n    ')
            stream.write(f'{spec} ')
            dumpAsScenicCode(getattr(self, prop), stream)

    def __str__(self):
        if hasattr(self, 'properties') and 'name' in self._propertiesSet:
            return self.name
        else:
            return f'unnamed {self.__class__.__name__}'

    def __repr__(self):
        if hasattr(self, 'properties'):
            allProps = { prop: getattr(self, prop) for prop in self.properties }
        else:
            allProps = '<under construction>'
        return f'{type(self).__name__}({allProps})'

## Mutators

class Mutator:
    """An object controlling how the :keyword:`mutate` statement affects an `Object`.

    A `Mutator` can be assigned to the ``mutator`` property of an `Object` to
    control the effect of the :keyword:`mutate` statement. When mutation is enabled
    for such an object using that statement, the mutator's `appliedTo` method
    is called to compute a mutated version. The `appliedTo` method can also decide
    whether to apply mutators inherited from superclasses.
    """

    def appliedTo(self, obj):
        """Return a mutated copy of the given object. Implemented by subclasses.

        The mutator may inspect the ``mutationScale`` attribute of the given object
        to scale its effect according to the scale given in ``mutate O by S``.

        Returns:
            A pair consisting of the mutated copy of the object (which is most easily
            created using `_copyWith`) together with a Boolean indicating whether the
            mutator inherited from the superclass (if any) should also be applied.
        """
        raise NotImplementedError

class PositionMutator(Mutator):
    """Mutator adding Gaussian noise to ``position``. Used by `Point`.

    Attributes:
        stddevs (tuple[float,float,float]): standard deviation of noise for each dimension (x,y,z).
    """
    def __init__(self, stddevs):
        self.stddevs = tuple(stddevs)

    def appliedTo(self, obj):
        noise = Vector( random.gauss(0, self.stddevs[0]*obj.mutationScale), 
                        random.gauss(0, self.stddevs[1]*obj.mutationScale),
                        random.gauss(0, self.stddevs[2]*obj.mutationScale))
        pos = obj.position + noise
        return (obj._copyWith(position=pos), True)      # allow further mutation

    def __eq__(self, other):
        if type(other) is not type(self):
            return NotImplemented
        return (other.stddevs == self.stddevs)

    def __hash__(self):
        return hash(self.stddevs)

class OrientationMutator(Mutator):
    """Mutator adding Gaussian noise to ``yaw``, ``pitch``, and ``roll``. Used by `OrientedPoint`.

    Attributes:
        stddevs (tuple[float,float,float]): standard deviation of noise for each angle (yaw, pitch, roll).
    """
    def __init__(self, stddevs):
        self.stddevs = tuple(stddevs)

    def appliedTo(self, obj):
        yaw   = obj.yaw   + random.gauss(0, self.stddevs[0] * obj.mutationScale)
        pitch = obj.pitch + random.gauss(0, self.stddevs[1] * obj.mutationScale)
        roll  = obj.roll  + random.gauss(0, self.stddevs[2] * obj.mutationScale)

        new_obj = obj._copyWith(yaw=yaw, pitch=pitch, roll=roll)

        return (new_obj, True)      # allow further mutation

    def __eq__(self, other):
        if type(other) is not type(self):
            return NotImplemented
        return (other.stddevs == self.stddevs)

    def __hash__(self):
        return hash(self.stddevs)

## Point

class Point(Constructible):
    """The Scenic base class ``Point``.

    The default mutator for `Point` adds Gaussian noise to ``position`` with
    a standard deviation given by the ``positionStdDev`` property.

    Properties:
        position (`Vector`; dynamic): Position of the point. Default value is the origin (0,0,0).
        width (float): Default value 0 (only provided for compatibility with
          operators that expect an `Object`).
        length (float): Default value 0.
        height (float): Default value 0.
        baseOffset (`Vector`): An offset indicating where the base of this `Point` is relative to
          its position. Used by the :keyword:`on` specifier. Default value is (0,0,0).
        contactTolerance (float): The maximum distance can be away from a surface to be considered
          on the surface. Instances are placed at this distance away from a point when the 
          :keyword:`on` specifier is used. Default value 0.
        onDirection (`Vector`): The direction used to determine where to place
          this `Point` on a region, when using the modifying :keyword:`on` specifier.
          See the :sampref:`on {region}` page for more details. Default value is None,
          indicating the direction will be inferred from the region this object is being placed on.
        visibleDistance (float): Distance used to determine the visible range of this object.
          Default value 50.
        viewRayDensity (float): By default determines the number of rays used during visibility checks. 
          This value is the density of rays per degree of visible range in one dimension. The total
          number of rays sent will be this value squared per square degree of this object's view angles. 
          This value determines the default value for viewRayCount, so if viewRayCount is overwritten this value is ignored.
          Default value 5.
        mutationScale (float): Overall scale of mutations, as set by the
          :keyword:`mutate` statement. Default value 0 (mutations disabled).
        positionStdDev (tuple[float, float, float]): Standard deviation of Gaussian noise 
          for each dimension (x,y,z) to be added to this object's :prop:`position` 
          when mutation is enabled with scale 1. Default value (1,1,0), mutating only the x,y values
          of the point.
    """
    _scenic_properties = {
        "position": PropertyDefault((), {'dynamic'}, lambda self: Vector(0.0, 0.0, 0.0)),
        "width": 0.0,
        "length": 0.0,
        "height": 0.0,

        "baseOffset": Vector(0,0,0),
        "contactTolerance": 0,
        "onDirection": None,

        # This property is defined in OrientedPoint, but we provide a default value
        # for Points for implementation convenience.
        "viewAngles": (math.tau, math.pi),
        "visibleDistance": 50,
        "viewRayDensity": 5,
        "viewRayCount": None,
        "viewRayDistanceScaling": False,

        "mutationScale": 0,
        "mutator": PropertyDefault(('positionStdDev', ), {'additive'},
                                lambda self: PositionMutator((self.positionStdDev))),
        "positionStdDev": (1,1,0),

        # This property is defined in Object, but we provide a default empty value
        # for Points for implementation convenience.
        "regionContainedIn": None,

        # These properties are used internally to store entities that must be able to
        # or must be unable to observe (view) this entity.
        "_observingEntity": None,
        "_nonObservingEntity": None,
    }

    @cached_property
    def visibleRegion(self):
        """The :term:`visible region` of this object.

        The visible region of a `Point` is a sphere centered at its ``position`` with
        radius ``visibleDistance``.
        """
        dimensions = (self.visibleDistance, self.visibleDistance, self.visibleDistance)
        return SpheroidRegion(position=self.position, dimensions=dimensions)

    @lru_cache(maxsize=None)
    def canSee(self, other, occludingObjects=list(), debug=False) -> bool:
        """Whether or not this `Point` can see ``other``.

        Args:
            other: A `Point`, `OrientedPoint`, or `Object` to check
              for visibility.
            occludingObjects: A list of objects that can occlude visibility.
        """
        return canSee(position=self.position, orientation=None, visibleDistance=self.visibleDistance, \
            viewAngles=(math.tau, math.pi), rayCount=self.viewRayCount, rayDensity=self.viewRayDensity, 
            distanceScaling=self.viewRayDistanceScaling, target=other, occludingObjects=occludingObjects, debug=debug)

    @cached_property
    def corners(self):
        return (self.position,)

    def toVector(self) -> Vector:
        return self.position

    def sampleGiven(self, value):
        sample = super().sampleGiven(value)
        if self.mutationScale != 0:
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
    """The Scenic class ``OrientedPoint``.

    The default mutator for `OrientedPoint` adds Gaussian noise to ``heading``
    with a standard deviation given by the ``headingStdDev`` property, then
    applies the mutator for `Point`.

    Properties:
        yaw (float; dynamic): Yaw of the `OrientedPoint` in radians in the local coordinate system
          provided by :prop:`parentOrientation`. Default value 0.
        pitch (float; dynamic): Pitch of the `OrientedPoint` in radians in the local coordinate system
          provided by :prop:`parentOrientation`. Default value 0.
        roll (float; dynamic): Roll of the `OrientedPoint` in radians in the local coordinate system
          provided by :prop:`parentOrientation`. Default value 0.
        parentOrientation (`Orientation`): The local coordinate system that the `OrientedPoint`'s
          :prop:`yaw`, :prop:`pitch`, and :prop:`roll` are interpreted in. Default 
          value is the global coordinate system, where an object is flat in the XY plane,
          facing North.
        orientation (`Orientation`; dynamic; final): The orientation of the `OrientedPoint` relative
          to the global coordinate system. Derived from the :prop:`yaw`, :prop:`pitch`, 
          :prop:`roll`, and :prop:`parentOrientation` of this `OrientedPoint` and non-overridable.
        heading (float; dynamic; final): Yaw value of this `OrientedPoint` in the global coordinate
          system. Derived from :prop:`orientation` and non-overridable.
        viewAngles (tuple[float,float]): Horizontal and vertical view angles of this `OrientedPoint`
          in radians. Horizontal view angle can be up to 2π and vertical view angle can be
          up to π. Values greater than these will be truncated. Default value is (2π, π)
        orientationStdDev (float): Standard deviation of Gaussian noise to add to this
          object's Euler angles (yaw, pitch, roll) when mutation is enabled with scale 1.
          Default value (5π, 0, 0), mutating only the :prop:`yaw` of this `OrientedPoint`.
    """
    _scenic_properties = {
        'yaw': PropertyDefault((), {'dynamic'}, lambda self: 0),
        'pitch': PropertyDefault((), {'dynamic'}, lambda self: 0),
        'roll': PropertyDefault((), {'dynamic'}, lambda self: 0),
        'parentOrientation': Orientation.fromEuler(0, 0, 0),

        'orientation': PropertyDefault(
            {'yaw', 'pitch', 'roll', 'parentOrientation'},
            {'dynamic', 'final'},
            lambda self: (Orientation.fromEuler(self.yaw, self.pitch, self.roll)
                      * self.parentOrientation)
        ),
        'heading': PropertyDefault({'orientation'}, {'dynamic', 'final'},
            lambda self: self.yaw if alwaysGlobalOrientation(self.parentOrientation) else self.orientation.yaw),

        'viewAngle': math.tau, # Primarily for backwards compatibility. Set viewAngles instead.
        'viewAngles': PropertyDefault(('viewAngle',), set(), lambda self: (self.viewAngle, math.pi)),

        'mutator': PropertyDefault({'orientationStdDev'}, {'additive'},
            lambda self: OrientationMutator(self.orientationStdDev)),
        'orientationStdDev': (math.radians(5),0,0),
    }

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        if self.viewAngles[0] > math.tau or self.viewAngles[1] > math.pi:
            warnings.warn("ViewAngles can not have values greater than (math.tau, math.pi). Truncating values...")
            self.viewAngles = (min(self.viewAngles[0], math.tau), min(self.viewAngles[1], math.pi))

    @cached_property
    def visibleRegion(self):
        """The :term:`visible region` of this object.

        The visible region of an `OrientedPoint` restricts that of `Point` (a sphere with
        radius :prop:`visibleDistance`) based on the value of :prop:`viewAngles`. In
        general, it is a capped rectangular pyramid subtending an angle of
        :scenic:`viewAngles[0]` horizontally and :scenic:`viewAngles[1]` vertically, as
        long as those angles are less than π/2; larger angles yield various kinds of
        wrap-around regions. See `DefaultViewRegion` for details.
        """
        return DefaultViewRegion(visibleDistance=self.visibleDistance, viewAngles=self.viewAngles,\
            position=self.position, rotation=self.orientation)

    @lru_cache(maxsize=None)
    def canSee(self, other, occludingObjects=list(), debug=False) -> bool:
        """Whether or not this `OrientedPoint` can see ``other``.

        Args:
            other: A `Point`, `OrientedPoint`, or `Object` to check
              for visibility.
            occludingObjects: A list of objects that can occlude visibility.
        """
        return canSee(position=self.position, orientation=self.orientation, visibleDistance=self.visibleDistance,
            viewAngles=self.viewAngles, rayCount=self.viewRayCount, rayDensity=self.viewRayDensity, 
            distanceScaling=self.viewRayDistanceScaling, target=other, occludingObjects=occludingObjects, debug=debug)

    def relativize(self, vec):
        pos = self.relativePosition(vec)
        return OrientedPoint._with(position=pos, parentOrientation=self.orientation)

    def relativePosition(self, vec):
        return self.position.offsetRotated(self.orientation, vec)

    def distancePast(self, vec):
        """Distance past a given point, assuming we've been moving in a straight line."""
        diff = self.position - vec
        return diff.rotatedBy(-self.heading).y

    def toHeading(self) -> float:
        return self.heading

    def toOrientation(self) -> Orientation:
        return self.orientation

## Object

class Object(OrientedPoint):
    """The Scenic class ``Object``.

    This is the default base class for Scenic classes.

    Properties:
        width (float): Width of the object, i.e. extent along its X axis.
          Default value of 1 inherited from the object's :prop:`shape`.
        length (float): Length of the object, i.e. extent along its Y axis.
          Default value of 1 inherited from the object's :prop:`shape`.
        height (float): Height of the object, i.e. extent along its Z axis.
          Default value of 1 inherited from the object's :prop:`shape`.
        shape (`Shape`): The shape of the object, which must be an instance of `Shape`.
          The default shape is a box, with default unit dimensions.
        allowCollisions (bool): Whether the object is allowed to intersect
          other objects. Default value ``False``.
        regionContainedIn (`Region` or ``None``): A `Region` the object is
          required to be contained in. If ``None``, the object need only be
          contained in the scenario's :term:`workspace`.
        baseOffset (`Vector`): An offset from the :prop:`position` of the Object
          to the base of the object, used by the :keyword:`on` specifier. Default value
          is :scenic:`(0, 0, -self.height/2)`, placing the base of the Object at the bottom
          center of the Object's bounding box.
        contactTolerance (float): The distance away an object should be placed
          from a point to be on the point, used by the :keyword:`on` specifier. Default
          value 1e-4.
        sideComponentThresholds (`DimensionLimits`): Used to determine the
          various sides of an object (when using the default implementation). 
          The three interior 2-tuples represent the maximum and minimum bounds
          for each dimension's (x,y,z) surface. See `defaultSideSurface` for details.
          Default value :scenic:`((-0.5, 0.5), (-0.5, 0.5), (-0.5, 0.5))`.
        cameraOffset (`Vector`): Position of the camera for the :keyword:`can see`
          operator, relative to the object's :prop:`position`. Default :scenic:`(0, 0, 0)`.
        requireVisible (bool): Whether the object is required to be visible
          from the ``ego`` object. Default value ``False``.
        occluding (bool): Whether or not this object can occlude other objects. Default
          value ``True``.
        color (tuple[float, float, float] or `None`): An optional color property that is
          used by the internal visualizer, or possibly simulators. All values should
          be between 0 and 1. Default value ``None``
        velocity (`Vector`; *dynamic*): Velocity in dynamic simulations. Default value is
          the velocity determined by :prop:`speed` and :prop:`orientation`.
        speed (float; dynamic): Speed in dynamic simulations. Default value 0.
        angularVelocity (`Vector`; *dynamic*): 
        angularSpeed (float; dynamic): Angular speed in dynamic simulations. Default
          value 0.
        behavior: Behavior for dynamic agents, if any (see :ref:`dynamics`). Default
          value ``None``.
        lastActions: Tuple of :term:`actions` taken by this agent in the last time step
          (or `None` if the object is not an agent or this is the first time step).
    """
    _scenic_properties = {
        'width': PropertyDefault(('shape',), {}, lambda self: self.shape.width),
        'length': PropertyDefault(('shape',), {}, lambda self: self.shape.length),
        'height': PropertyDefault(('shape',), {}, lambda self: self.shape.height),
        'shape': BoxShape(),

        'allowCollisions': False,
        'regionContainedIn': None,
        'baseOffset': PropertyDefault(('height',), {}, lambda self: Vector(0, 0, -self.height/2)),
        'contactTolerance': 1e-4,
        'sideComponentThresholds': ((-0.5, 0.5), (-0.5, 0.5), (-0.5, 0.5)),

        'cameraOffset': Vector(0, 0, 0),
        'requireVisible': False,
        'occluding': True,

        "color": None,

        'velocity': PropertyDefault((), {'dynamic'}, lambda self: Vector(0, 0, 0)),
        'speed': PropertyDefault((), {'dynamic'}, lambda self: 0),
        'angularVelocity': PropertyDefault((), {'dynamic'}, lambda self: Vector(0, 0, 0)),
        'angularSpeed': PropertyDefault((), {'dynamic'}, lambda self: 0),

        "behavior": None,
        "lastActions": None,
    }

    def __new__(cls, *args, **kwargs):
        obj = super().__new__(cls, *args, **kwargs)
        # The _dynamicProxy attribute stores a mutable copy of the object used during
        # simulations, intercepting all attribute accesses to the original object;
        # we set this attribute very early to prevent problems during unpickling.
        object.__setattr__(obj, '_dynamicProxy', obj)
        return obj

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.hw = hw = self.width / 2
        self.hl = hl = self.length / 2
        self.hh = hh = self.height / 2
        self.radius = hypot(hw, hl, hh) # circumcircle; for collision detection

        self._relations = []

    @classmethod
    def _specify(cls, context, prop, value):
        # Normalize types of some built-in properties
        if prop == 'behavior' and value != None:
            import scenic.syntax.veneer as veneer   # TODO improve?
            value = toType(value, veneer.Behavior,
                           f'"behavior" of {cls.__name__} not a behavior')
        super()._specify(context, prop, value)

    def _register(self):
        import scenic.syntax.veneer as veneer   # TODO improve?
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

    @lru_cache(maxsize=None)
    def containsPoint(self, point):
        """ Whether or not the space this object occupies contains a point"""
        return self.occupiedSpace.containsPoint(point)

    @lru_cache(maxsize=None)
    def distanceTo(self, point):
        """ The minimal distance from the space this object occupies to a given point"""
        return self.occupiedSpace.distanceTo(point)

    @lru_cache(maxsize=None)
    def intersects(self, other):
        """ Whether or not this object intersects another object"""
        # For objects that are boxes and flat, we can take a fast route
        if self._isPlanarBox and other._isPlanarBox:
            if abs(self.position.z - other.position.z) > (self.height + other.height) / 2:
                return False

            self_poly = self._boundingPolygon
            other_poly = other._boundingPolygon
            return self_poly.intersects(other_poly)

        if isLazy(self.occupiedSpace) or isLazy(other.occupiedSpace):
            raise RandomControlFlowError("Cannot compute intersection between Objects with non-fixed values.")

        return self.occupiedSpace.intersects(other.occupiedSpace)

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
    def top(self):
        return self.relativize(Vector(0, 0, self.hh))

    @cached_property
    def bottom(self):
        return self.relativize(Vector(0, 0, -self.hh))

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
    def topFrontLeft(self):
        return self.relativize(Vector(-self.hw, self.hl, self.hh))

    @cached_property
    def topFrontRight(self):
        return self.relativize(Vector(self.hw, self.hl, self.hh))

    @cached_property
    def topBackLeft(self):
        return self.relativize(Vector(-self.hw, -self.hl, self.hh))

    @cached_property
    def topBackRight(self):
        return self.relativize(Vector(self.hw, -self.hl, self.hh))

    @cached_property
    def bottomFrontLeft(self):
        return self.relativize(Vector(-self.hw, self.hl, -self.hh))

    @cached_property
    def bottomFrontRight(self):
        return self.relativize(Vector(self.hw, self.hl, -self.hh))

    @cached_property
    def bottomBackLeft(self):
        return self.relativize(Vector(-self.hw, -self.hl, -self.hh))

    @cached_property
    def bottomBackRight(self):
        return self.relativize(Vector(self.hw, -self.hl, -self.hh))

    @cached_property
    def visibleRegion(self):
        """The :term:`visible region` of this object.

        The visible region of an `Object` is the same as that of an `OrientedPoint` (see
        `OrientedPoint.visibleRegion`) except that it is offset by the value of
        :prop:`cameraOffset` (which is the zero vector by default).
        """
        true_position = self.position.offsetRotated(self.orientation, self.cameraOffset)
        return DefaultViewRegion(visibleDistance=self.visibleDistance,
                                 viewAngles=self.viewAngles,
                                 position=true_position, rotation=self.orientation)

    @lru_cache(maxsize=None)
    def canSee(self, other, occludingObjects=list(), debug=False) -> bool:
        """Whether or not this `Object` can see ``other``.

        Args:
            other: A `Point`, `OrientedPoint`, or `Object` to check
              for visibility.
            occludingObjects: A list of objects that can occlude visibility.
        """
        true_position = self.position.offsetRotated(self.orientation, self.cameraOffset)
        return canSee(position=true_position, orientation=self.orientation, visibleDistance=self.visibleDistance, \
            viewAngles=self.viewAngles, rayCount=self.viewRayCount, rayDensity=self.viewRayDensity, 
            distanceScaling=self.viewRayDistanceScaling, target=other, occludingObjects=occludingObjects, debug=debug)

    @cached_property
    def corners(self):
        """A tuple containing the corners of this object's bounding box"""
        hw, hl, hh = self.hw, self.hl, self.hh
        # Note: 2D show method assumes cyclic order of first 4 vertices
        return (
            self.relativePosition(Vector(hw,  hl,  hh)),
            self.relativePosition(Vector(-hw, hl,  hh)),
            self.relativePosition(Vector(-hw, -hl, hh)),
            self.relativePosition(Vector(hw,  -hl, hh)),
            self.relativePosition(Vector(hw,  hl,  -hh)),
            self.relativePosition(Vector(-hw, hl,  -hh)),
            self.relativePosition(Vector(-hw, -hl, -hh)),
            self.relativePosition(Vector(hw,  -hl, -hh)),
        )

    @cached_property
    def occupiedSpace(self):
        """A region representing the space this object occupies"""
        return MeshVolumeRegion(mesh=self.shape.mesh, \
                dimensions=(self.width, self.length, self.height), \
                position=self.position, rotation=self.orientation)

    @property
    def _isConvex(self):
        """Whether this object's shape is convex"""
        return self.shape.isConvex

    @property
    def _hasStaticBounds(self):
        deps = (
            self.position, self.orientation, self.shape,
            self.width, self.length, self.height,
        )
        return not any(needsSampling(v) for v in deps)

    @cached_property
    def boundingBox(self):
        """A region representing this object's bounding box"""
        return MeshVolumeRegion(self.occupiedSpace.mesh.bounding_box, centerMesh=False)

    @cached_property
    def inradius(self):
        """A lower bound on the inradius of this object"""
        # First check if all needed variables are defined. If so, we can
        # compute the inradius exactly.
        width, length, height = self.width, self.length, self.height
        shape = self.shape
        if not any(needsSampling(val) for val in (width, length, height, shape)):
            shapeRegion = MeshVolumeRegion(mesh=shape.mesh,
                                           dimensions=(width, length, height))
            return shapeRegion.inradius

        # If we havea uniform distribution over shapes and a supportInterval for each dimension,
        # we can compute a supportInterval for this object's inradius

        # Define helper class
        class InradiusHelper:
            def __init__(self, support):
                self.support = support

            def supportInterval(self):
                return self.support

        # Extract bounds on all dimensions
        min_width, max_width = supportInterval(width)
        min_length, max_length = supportInterval(length)
        min_height, max_height = supportInterval(height)

        if any(val == None for val in [min_width, max_width, 
                                       min_length, max_length,
                                       min_height, max_height]):
            # Can't get a bound on one or more dimensions, abort
            return 0

        min_bounds = np.array([min_width, min_length, min_height])
        max_bounds = np.array([max_width, max_length, max_height])

        # Extract a list of possible shapes
        if isinstance(shape, Shape):
            shapes = [shape]
        elif isinstance(shape, MultiplexerDistribution):
            if all(isinstance(opt, Shape) for opt in shape.options):
                shapes = shape.options
            else:
                # Something we don't recognize, abort
                return 0

        # Check that all possible shapes contain the origin
        if not all(shape.containsCenter for shape in shapes):
            # One or more shapes has inradius 0
            return 0

        # Get the inradius for each shape with the min and max bounds
        min_distances = [MeshVolumeRegion(mesh=shape.mesh, dimensions=min_bounds).inradius for shape in shapes]
        max_distances = [MeshVolumeRegion(mesh=shape.mesh, dimensions=max_bounds).inradius for shape in shapes]

        distance_range = (min(min_distances), max(max_distances))

        return InradiusHelper(support=distance_range)

    @cached_property
    def surface(self):
        """A region containing the entire surface of this object"""
        return self.occupiedSpace.getSurfaceRegion()

    @cached_property
    def onSurface(self):
        """The surface used by the ``on`` specifier.

        This region is used to sample position when
        another object is placed ``on`` this object. By default
        the top surface of this object (`topSurface`), but can
        be overwritten by subclasses.
        """
        return self.topSurface

    @cached_property
    def rightSurface(self):
        """A region containing the right surface of this object"""
        return defaultSideSurface(self.occupiedSpace, dimension=0, \
            positive=True, thresholds=self.sideComponentThresholds)

    @cached_property
    def leftSurface(self):
        """A region containing the left surface of this object"""
        return defaultSideSurface(self.occupiedSpace, dimension=0, \
            positive=False, thresholds=self.sideComponentThresholds)

    @cached_property
    def frontSurface(self):
        """A region containing the front surface of this object"""
        return defaultSideSurface(self.occupiedSpace, dimension=1, \
            positive=True, thresholds=self.sideComponentThresholds)

    @cached_property
    def backSurface(self):
        """A region containing the back surface of this object"""
        return defaultSideSurface(self.occupiedSpace, dimension=1, \
            positive=False, thresholds=self.sideComponentThresholds)

    @cached_property
    def topSurface(self):
        """A region containing the top surface of this object"""
        return defaultSideSurface(self.occupiedSpace, dimension=2, \
            positive=True, thresholds=self.sideComponentThresholds)

    @cached_property
    def bottomSurface(self):
        """A region containing the bottom surface of this object"""
        return defaultSideSurface(self.occupiedSpace, dimension=2, \
            positive=False, thresholds=self.sideComponentThresholds)

    def show3D(self, viewer, highlight=False):
        if needsSampling(self):
            raise RuntimeError('tried to show() symbolic Object')

        # Render the object
        object_mesh = self.occupiedSpace.mesh.copy()

        if highlight:
            object_mesh.visual.face_colors = [30, 179, 0, 255]
        elif self.color is not None:
            object_mesh.visual.face_colors = self.color

        viewer.add_geometry(object_mesh)

        # If the camera is not a sphere, render the visible pyramid as a blue wireframe
        if self.viewAngles != (math.tau, math.pi) or self.visibleDistance != 50:
            camera_pyramid_mesh = self.visibleRegion.mesh.copy()

            edges = camera_pyramid_mesh.face_adjacency_edges[camera_pyramid_mesh.face_adjacency_angles > np.radians(0.1)].copy()
            vertices = camera_pyramid_mesh.vertices.copy()

            edge_path = trimesh.path.Path3D(**trimesh.path.exchange.misc.edges_to_path(edges, vertices))

            edge_path.colors = [[30, 30, 150, 255] for _ in range(len(edge_path.entities))]

            viewer.add_geometry(edge_path)

    def show2D(self, workspace, plt, highlight=False):
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

        corners = [workspace.scenicToSchematicCoords(corner) for corner in self.corners[:4]]
        x, y = zip(*corners)
        color = self.color if hasattr(self, 'color') else (1, 0, 0)
        plt.fill(x, y, color=color)

        frontMid = averageVectors(corners[0], corners[1])
        baseTriangle = [frontMid, corners[2], corners[3]]
        triangle = [averageVectors(p, spos, weight=0.5) for p in baseTriangle]
        x, y = zip(*triangle)
        plt.fill(x, y, "w")
        plt.plot(x + (x[0],), y + (y[0],), color="k", linewidth=1)

    @cached_property
    def _isPlanarBox(self):
        """Whether this object is a box aligned with the XY plane."""
        return (isinstance(self.shape, BoxShape)
                and self.orientation.pitch == 0
                and self.orientation.roll == 0)

    @cached_property
    def _boundingPolygon(self):
        # Fast case for planar boxes
        if self._isPlanarBox:
            width, length = self.width, self.length
            pos = self.position
            yaw = self.orientation.yaw
            cyaw, syaw = math.cos(yaw), math.sin(yaw)
            matrix = [width*cyaw, -length*syaw, width*syaw, length*cyaw, pos[0], pos[1]]
            return shapely.affinity.affine_transform(_unitBox, matrix)

        return self.occupiedSpace._boundingPolygon

_unitBox = shapely.geometry.Polygon(((0.5, 0.5), (-0.5, 0.5), (-0.5, -0.5), (0.5, -0.5)))

@distributionFunction
def defaultSideSurface(occupiedSpace, dimension, positive, thresholds):
    """Extracts a side surface from the occupiedSpace of an object.

    This function is the default implementation for computing a region
    representing a side surface of an object. This is done by keeping only the
    faces of the object's ``occupiedSpace`` mesh that have normal 
    vectors with a large/small enough x,y, or z component. For example, 
    for the front surface of an object we would would keep all faces that 
    had a normal vector with y component greater than ``thresholds[1][1]`` 
    and for the back surface of an object we would keep all faces that
    had a normal vector with y component less than ``thresholds[1][0]``.

    Args:
        occupiedSpace: The `occupiedSpace` region of the object to
          extract the side surface from.
        dimension: The target dimension who's component will be checked.
        positive: If `False`, the target component must be less than 
          the first value in the appropriate tuple. If `True`, the
          component must be greater than the second value in the
          appropriate tuple.
        thresholds: A 3-tuple of 2-tuples, one for each dimension (x,y,z),
          with each tuple containing the thresholds for a positive and
          non-positive side in each dimension.
        on_dimension: The on_dimension to be passed to the created surface.
    """
    # Extract mesh from object
    obj_mesh = occupiedSpace.mesh.copy()

    # Extract appropriate thresholds
    threshold = thresholds[dimension][int(positive)]

    # Drop all faces whose normal vector do not have a sufficiently
    # large component.
    face_normal_vals = obj_mesh.face_normals[:, dimension]
    if positive:
        face_mask = face_normal_vals >= threshold
    else:
        face_mask = face_normal_vals <= threshold

    obj_mesh.faces = obj_mesh.faces[face_mask]
    obj_mesh.remove_unreferenced_vertices()

    # Check if the resulting surface is empty and return an appropriate region.
    if not obj_mesh.is_empty:
        return MeshSurfaceRegion(mesh=obj_mesh, centerMesh=False)
    else:
        return EmptyRegion(name="EmptyTopSurface")

def canSee(position, orientation, visibleDistance, viewAngles, 
            rayCount, rayDensity, distanceScaling,\
            target, occludingObjects, debug=False):
    """Perform visibility checks on Points, OrientedPoints, or Objects, accounting for occlusion.

    For visibilty of Objects:

    1. Do several quick checks to see if the object is naively visible
       or not visible:

       * If the object contains its position and its position is visible,
         the object is visible.
       * If the viewer is inside the object, the object is visible.
       * If the closest distance from the object to the viewer is greater
         than the visible distance, the object is not visible.

    2. Check if the object crosses the back and/or front of the viewing
       object.
    3. Compute the spherical coordinates of all vertices in the mesh of
       the region we are trying to view, with the goal of using this to
       send rays only where they have a chance of hitting the region.
    4. Compute 2 ranges of angles (horizontal/vertical) in which rays have
       a chance of hitting the object, as follows:

       * If the object does not cross behind the viewer, take the min and
         max of the the spherical coordinate angles, while noting that this range
         is centered on the front of the viewer.
       * If the object crosses behind the viewer but not in front, transform 
         the spherical angles so they are coming from the back of the object,
         while noting that this range is centered on the back of the object.
       * If it crosses both, we do not optimize the amount of rays sent.

    5. Compute the intersection of the optimizated range from step 4 and
       the viewAngles range, accounting for where the optimization range is centered. 
       If it is empty, the object cannot be visible. If it is not empty, shoot rays at 
       the desired density in the intersection region. Keep all rays that intersect
       the object (candidate rays).
    6. If there are no candidate rays, the object is not visible.
    7. For each occluding object in occludingObjects: check if any candidate rays 
       intersect the occluding object at a distance less than the distance they intersected
       the target object. If they do, remove them from the candidate rays.
    8. If any candidate rays remain, the object is visible. If not, it is occluded
       and not visible.

    For visibility of Points/OrientedPoints:

    1. Check if distance from the viewer to the point is greater than visibleDistance.
       If so, the point cannot be visible
    2. Create a single candidate ray, using the vector from the viewer to the target.
       If this ray is outside of the bounds of viewAngles, the point cannot be visible.
    3. For each occluding object in occludingObjets: check if the candidate ray hits
       the occluding object at a distance less than the distance from the viewer to the
       target point. If so, then the object is not visible. Otherwise, the object is visible.

    Args:
        position: Position of the viewer, accounting for any offsets.
        orientation: Orientation of the viewer.
        visibleDistance: The maximum distance the viewer can view objects from.
        viewAngles: The horizontal and vertical view angles, in radians, of the viewer.
        rayCount: The total number of rays in each dimension used in visibility calculations..
        target: The target being viewed. Currently supports Point, OrientedPoint, and Object.
        occludingObjects: An optional list of objects which can occlude the target.
    """
    occludingObjects = list(occludingObjects)

    if rayCount is None:
        rayCount = (math.degrees(viewAngles[0])*rayDensity, math.degrees(viewAngles[1])*rayDensity)

        if distanceScaling:
            target_distance = target.position.distanceTo(position)

            rayCount = (rayCount[0]*viewAngles[0]*target_distance, rayCount[1]*viewAngles[1]*target_distance)

        altitudeScaling = True
    else:
        # Do not scale ray counts with altitude or distance if explicitly given
        altitudeScaling = False

    if isinstance(target, (Region, Object)):
        # Extract the target region from the object or region.
        if isinstance(target, Region):
            raise NotImplementedError
        elif isinstance(target, Object):
            # If the object contains its center and we can see the center, the object
            # is visible.
            if target.shape.containsCenter and \
                canSee(position, orientation, visibleDistance, viewAngles,
                       rayCount, rayDensity, distanceScaling,
                       target.position, occludingObjects):
                return True
            target_region = target.occupiedSpace

        # Check that the distance to the target is not greater than visibleDistance, and
        # see if position is in the target.
        if target.containsPoint(position):
            return True

        if target.distanceTo(position) > visibleDistance:
            return False

        # Orient the object so that it has the same relative position and orientation to the
        # origin as it did to the viewer
        target_vertices = target_region.mesh.vertices - np.array(position.coordinates)

        if orientation is not None:
            target_vertices = orientation.inverse.getRotation().apply(target_vertices)

        # Add additional points along each edge that could potentially have a higher altitude
        # than the endpoints.
        vec_1s = np.asarray(target_vertices[target_region.mesh.edges[:,0],:])
        vec_2s = np.asarray(target_vertices[target_region.mesh.edges[:,1],:])
        x1, y1, z1 = vec_1s[:,0], vec_1s[:,1], vec_1s[:,2]
        x2, y2, z2 = vec_2s[:,0], vec_2s[:,1], vec_2s[:,2]
        D = x1*x2 + y1*y2
        N = (x1**2 + y1**2)*z2 - D*z1
        M = (x2**2 + y2**2)*z1 - D*z2
        with np.errstate(divide='ignore', invalid='ignore'):
            t_vals = N/(N+M) # t values that can be an altitude local optimum

        # Keep only points where the t_value is between 0 and 1
        t_mask = np.logical_and(t_vals > 0, t_vals < 1)
        interpolated_points = vec_1s[t_mask] + t_vals[t_mask][:,None] * (vec_2s[t_mask] - vec_1s[t_mask])

        target_vertices = np.concatenate((target_vertices, interpolated_points), axis=0)

        ## Check if the object crosses the y axis ahead and/or behind the viewer

        # Extract the two vectors that are part of each edge crossing the y axis.
        y_cross_edges = (vec_1s[:,0]/vec_2s[:,0]) < 0
        vec_1s = vec_1s[y_cross_edges]
        vec_2s = vec_2s[y_cross_edges]

        # Figure out for which t value the vectors cross the y axis
        t = (-vec_1s[:,0])/(vec_2s[:,0]-vec_1s[:,0])

        # Figure out what the y value is when the y axis is crossed
        y_intercept_points = t*(vec_2s[:,1]-vec_1s[:,1]) + vec_1s[:,1]

        # If the object crosses ahead and behind the object, or through 0,
        # we will not optimize ray casting.
        target_crosses_ahead = np.any(y_intercept_points >= 0)
        target_crosses_behind = np.any(y_intercept_points <= 0)

        ## Compute the horizontal/vertical angle ranges which bound the object
        ## (from the origin facing forwards)
        spherical_angles = np.zeros((len(target_vertices[:,0]), 2))

        spherical_angles[:,0] = np.arctan2(target_vertices[:,1], target_vertices[:,0])
        spherical_angles[:,1] = np.arcsin(target_vertices[:,2]/ \
                                (np.linalg.norm(target_vertices, axis=1)))

        # Align azimuthal angle with y axis.
        spherical_angles[:,0] = spherical_angles[:,0] - math.pi/2

        # Normalize angles between (-Pi,Pi)
        spherical_angles[:,0] = np.mod(spherical_angles[:,0] + np.pi, 2*np.pi) - np.pi
        spherical_angles[:,1] = np.mod(spherical_angles[:,1] + np.pi, 2*np.pi) - np.pi

        # First we check if the vertical angles overlap with the vertical view angles.
        # If not, then the object cannot be visible.
        if np.min(spherical_angles[:,1]) > viewAngles[1]/2 or \
           np.max(spherical_angles[:,1]) < -viewAngles[1]/2:
            return False 

        ## Compute which horizontal/vertical angle ranges to cast rays in
        if target_crosses_ahead and target_crosses_behind:
            # No optimizations feasible here. Just send all rays.
            h_range = (-viewAngles[0]/2, viewAngles[0]/2)
            v_range = (-viewAngles[1]/2, viewAngles[1]/2)

            view_ranges = [(h_range, v_range)]

        elif target_crosses_behind:
            # We can keep the view angles oriented around the front of the object and
            # consider the spherical angles oriented around the back of the object.
            # We can then check for impossible visibility/optimize which rays will be cast.

            # Extract the viewAngle ranges
            va_h_range = (-viewAngles[0]/2, viewAngles[0]/2)
            va_v_range = (-viewAngles[1]/2, viewAngles[1]/2)

            # Convert spherical angles to be centered around the back of the viewing object.
            left_points  = spherical_angles[:,0] >= 0
            right_points = spherical_angles[:,0] < 0

            spherical_angles[:,0][left_points]  = spherical_angles[:,0][left_points] - np.pi
            spherical_angles[:,0][right_points] = spherical_angles[:,0][right_points] + np.pi

            sphere_h_range = (np.min(spherical_angles[:,0]), np.max(spherical_angles[:,0]))
            sphere_v_range = (np.min(spherical_angles[:,1]), np.max(spherical_angles[:,1]))

            # Extract the overlapping ranges in the horizontal and vertical view angles.
            # Note that the spherical range must cross the back plane and the view angles 
            # must cross the front plane (and are centered on these points),
            # which means we can just add up each side of the ranges and see if they add up to
            # greater than or equal to Pi. If none do, then it's impossible for object to overlap
            # with the viewAngle range.

            # Otherwise we can extract the overlapping v_ranges and use those going forwards.
            overlapping_v_range = (np.clip(sphere_v_range[0], va_v_range[0], va_v_range[1]),
                                   np.clip(sphere_v_range[1], va_v_range[0], va_v_range[1]))
            view_ranges = []

            if (abs(va_h_range[0]) + abs(sphere_h_range[1]) > math.pi):
                h_range = (va_h_range[0], -math.pi+sphere_h_range[1])
                view_ranges.append((h_range, overlapping_v_range))

            if (abs(va_h_range[1]) + abs(sphere_h_range[0]) > math.pi):
                h_range = (math.pi+sphere_h_range[0], va_h_range[1])
                view_ranges.append((h_range, overlapping_v_range))

            if len(view_ranges) == 0:
                return False

        else:
            # We can immediately check for impossible visbility/optimize which rays
            # will be cast.

            # Check if view range and spherical angles overlap in horizontal or
            # vertical dimensions. If not, return False
            if  (np.max(spherical_angles[:,0]) < -viewAngles[0]/2) or \
                (np.min(spherical_angles[:,0]) >  viewAngles[0]/2) or \
                (np.max(spherical_angles[:,1]) < -viewAngles[1]/2) or \
                (np.min(spherical_angles[:,1]) >  viewAngles[1]/2):
                return False

            # Compute trimmed view angles
            h_min = np.clip(np.min(spherical_angles[:,0]), -viewAngles[0]/2, viewAngles[0]/2)
            h_max = np.clip(np.max(spherical_angles[:,0]), -viewAngles[0]/2, viewAngles[0]/2)
            v_min = np.clip(np.min(spherical_angles[:,1]), -viewAngles[1]/2, viewAngles[1]/2)
            v_max = np.clip(np.max(spherical_angles[:,1]), -viewAngles[1]/2, viewAngles[1]/2)

            h_range = (h_min, h_max)
            v_range = (v_min, v_max)

            view_ranges = [(h_range, v_range)]

        ## Generate candidate rays
        candidate_ray_list = []

        for h_range, v_range in view_ranges:
            h_size = h_range[1] - h_range[0]
            v_size = v_range[1] - v_range[0]

            assert h_size > 0
            assert v_size > 0

            scaled_v_ray_count = math.ceil(v_size/(viewAngles[1]) * rayCount[1])
            v_angles = np.linspace(v_range[0],v_range[1], scaled_v_ray_count)

            # If altitudeScaling is true, we will scale the number of rays by the cosine of the altitude
            # to get a uniform spread.
            if altitudeScaling:
                h_ray_counts = np.maximum(np.ceil(np.cos(v_angles) * h_size/(viewAngles[0]) * rayCount[0]), 1).astype(int)
                h_angles_list = [np.linspace(h_range[0],h_range[1], h_ray_count) for h_ray_count in h_ray_counts]
                angle_matrices = [np.column_stack([h_angles_list[i], np.repeat([v_angles[i]], len(h_angles_list[i]))]) for i in range(len(v_angles))]
                angle_matrix = np.concatenate(angle_matrices, axis=0)
            else:
                scaled_h_ray_count = math.ceil(h_size/(viewAngles[0]) * rayCount[0])
                h_angles = np.linspace(h_range[0],h_range[1], scaled_h_ray_count)
                angle_matrix = np.column_stack([np.repeat(h_angles, len(v_angles)), np.tile(v_angles, len(h_angles))])

            ray_vectors = np.zeros((len(angle_matrix[:,0]), 3))

            ray_vectors[:,0] = -np.sin(angle_matrix[:,0])
            ray_vectors[:,1] = np.cos(angle_matrix[:,0])
            ray_vectors[:,2] = np.tan(angle_matrix[:,1])

            ray_vectors /= np.linalg.norm(ray_vectors, axis=1)[:, np.newaxis]
            candidate_ray_list.append(ray_vectors)

        ray_vectors = np.concatenate(candidate_ray_list, axis=0)

        if orientation is not None:
            ray_vectors = orientation.getRotation().apply(ray_vectors)
        
        ## DEBUG ##
        #Show all original candidate rays
        if debug:    
            vertices = [visibleDistance*vec + position.coordinates for vec in ray_vectors]
            vertices = [position.coordinates] + vertices
            lines = [trimesh.path.entities.Line([0,v]) for v in range(1,len(vertices))]
            colors =[(0,0,255,255) for line in lines]

            render_scene = trimesh.scene.Scene()
            render_scene.add_geometry(trimesh.path.Path3D(entities=lines, vertices=vertices, process=False, colors=colors))
            render_scene.add_geometry(target.occupiedSpace.mesh)
            for i in list(occludingObjects):
              render_scene.add_geometry(i.occupiedSpace.mesh)
            render_scene.show()

        # Shuffle the rays and split them into smaller batches, so we get the
        # opportunity to return early.
        ray_indices = np.arange(len(ray_vectors))
        rng = np.random.default_rng(seed=42)
        rng.shuffle(ray_indices)

        batch_size = 128

        # Use a generator to avoid having to split a large array
        def ray_indices_generator():
            bottom_index = 0
            top_index = batch_size

            while bottom_index < len(ray_vectors):
                yield ray_indices[bottom_index:top_index]

                bottom_index = top_index
                top_index += batch_size

        for target_ray_indices in ray_indices_generator():
            ray_batch = ray_vectors[target_ray_indices]

            # Check if candidate rays hit target
            raw_target_hit_info = target_region.mesh.ray.intersects_location(
                ray_origins=[position.coordinates for ray in ray_batch],
                ray_directions=ray_batch)

            # If no hits, this object can't be visible with these rays
            if len(raw_target_hit_info[0]) == 0:
                continue

            # Extract rays that are within visibleDistance, mapping the vector
            # to the closest distance at which they hit the target
            hit_locs = raw_target_hit_info[0]
            hit_distances = np.linalg.norm(hit_locs - np.array(position), axis=1)

            target_dist_map = {}

            for hit_iter in range(len(raw_target_hit_info[0])):
                hit_ray = tuple(ray_batch[raw_target_hit_info[1][hit_iter]])
                hit_dist = hit_distances[hit_iter]

                # If the hit was out of visible distance, don't consider it.
                if hit_dist > visibleDistance:
                    continue

                # If we don't already have a hit distance for this vector, or if
                # this hit was closer, update the target distance mapping.
                if hit_ray not in target_dist_map or hit_dist < target_dist_map[hit_ray]:
                    target_dist_map[hit_ray] = hit_dist

            # If no hits within range, this object can't be visible with these rays
            if len(target_dist_map) == 0:
                continue

            # Now check if occluded objects block sight to target
            candidate_rays = set(target_dist_map.keys())

            ## DEBUG ##
            #Show all candidate vertices that hit target
            if debug:            
                vertices = [visibleDistance*np.array(vec) + position.coordinates for vec in candidate_rays]
                vertices = [position.coordinates] + vertices
                lines = [trimesh.path.entities.Line([0,v]) for v in range(1,len(vertices))]
                colors =[(0,0,255,255) for line in lines]

                render_scene = trimesh.scene.Scene()
                render_scene.add_geometry(trimesh.path.Path3D(entities=lines, vertices=vertices, process=False, colors=colors))
                render_scene.add_geometry(target.occupiedSpace.mesh)
                for occ_obj in list(occludingObjects):
                    render_scene.add_geometry(occ_obj.occupiedSpace.mesh)
                render_scene.show()

            for occ_obj in occludingObjects:
                # If no more rays are candidates, then object is no longer visible.
                # Short circuit the loop to get to the next batch of rays.
                if len(candidate_rays) == 0:
                    break

                candidate_ray_list = np.array(list(candidate_rays))

                # Test all candidate rays against this occluding object
                object_hit_info = occ_obj.occupiedSpace.mesh.ray.intersects_location(
                    ray_origins=[position.coordinates for ray in candidate_ray_list],
                    ray_directions=candidate_ray_list)

                # If no hits, this object doesn't occlude. We don't need to
                # filter any rays for this object so move on the next object.
                if len(object_hit_info[0]) == 0:
                    continue

                # Check if any candidate ray hits the occluding object with a smaller
                # distance than the target.
                object_distances = np.linalg.norm(object_hit_info[0] - np.array(position), axis=1)

                occluded_rays = set()

                for hit_iter in range(len(object_hit_info[0])):
                    hit_ray = tuple(candidate_ray_list[object_hit_info[1][hit_iter]])
                    hit_dist = object_distances[hit_iter]

                    # If this ray hit the object earlier than it hit the target, reject the ray.
                    if hit_dist <= target_dist_map[hit_ray]:
                        occluded_rays.add(hit_ray)

                candidate_rays = candidate_rays - occluded_rays

                ## DEBUG ##
                # Show occluded and non occluded rays from this object
                if debug:
                    occluded_vertices = [visibleDistance*np.array(vec) + position.coordinates for vec in occluded_rays]
                    clear_vertices = [visibleDistance*np.array(vec) + position.coordinates for vec in candidate_rays]
                    vertices = occluded_vertices + clear_vertices
                    vertices = [position.coordinates] + vertices
                    lines = [trimesh.path.entities.Line([0,v]) for v in range(1,len(vertices))]
                    occluded_colors = [(255,0,0,255) for line in occluded_vertices]
                    clear_colors = [(0,255,0,255) for line in clear_vertices]
                    colors = occluded_colors + clear_colors
                    render_scene = trimesh.scene.Scene()
                    render_scene.add_geometry(trimesh.path.Path3D(entities=lines, vertices=vertices, process=False, colors=colors))
                    render_scene.add_geometry(target.occupiedSpace.mesh)
                    render_scene.add_geometry(occ_obj.occupiedSpace.mesh)
                    render_scene.show()

            if len(candidate_rays) > 0:
                return True

        # No rays hit the object and are not occluded, so the object is not visible
        return False

    elif isinstance(target, (Point, OrientedPoint, Vector)):
        if isinstance(target, (Point, OrientedPoint)):
            target_loc = target.position
        else:
            target_loc = target

        # First check if the distance to the point is less than or equal to the visible distance. If not, the object cannot
        # be visible.
        target_distance = position.distanceTo(target_loc)
        if target_distance > visibleDistance:
            return False

        # Create the single candidate ray and check that it's within viewAngles.
        target_vertex = target_loc - position
        candidate_ray = target_vertex/np.linalg.norm(target_vertex)

        azimuth = np.mod(np.arctan2(candidate_ray[1], candidate_ray[0]) - math.pi/2 + np.pi, 2*np.pi) - np.pi
        altitude = np.arcsin(candidate_ray[2]/(np.linalg.norm(candidate_ray)))

        if orientation is not None:
            relative_azimuth = azimuth - orientation.yaw
            relative_altitude = altitude - orientation.pitch
        else:
            relative_azimuth = azimuth
            relative_altitude = altitude

        candidate_ray_list = np.array([candidate_ray])

        ## DEBUG ##
        #Show all original candidate rays
        if debug:
            vertices = [visibleDistance*vec + position.coordinates for vec in candidate_ray_list]
            vertices = [position.coordinates] + vertices
            lines = [trimesh.path.entities.Line([0,v]) for v in range(1,len(vertices))]
            colors =[(0,0,255,255) for line in lines]

            render_scene = trimesh.scene.Scene()
            render_scene.add_geometry(trimesh.path.Path3D(entities=lines, vertices=vertices, process=False, colors=colors))
            render_scene.add_geometry(SpheroidRegion(position=target_loc, dimensions=(0.1,0.1,0.1)))
            for i in list(occludingObjects):
                render_scene.add_geometry(i.occupiedSpace.mesh)
            render_scene.show()

        # Check if this ray is within our view cone.
        if (not (-viewAngles[0]/2 <= relative_azimuth <= viewAngles[0]/2)) or \
            (not (-viewAngles[1]/2 <= relative_altitude <= viewAngles[1]/2)):
            return False

        # Now check if occluding objects block sight to target
        for occ_obj in occludingObjects:
            # Test all candidate rays against this occluding object
            object_hit_info = occ_obj.occupiedSpace.mesh.ray.intersects_location(
                ray_origins=[position.coordinates for ray in candidate_ray_list],
                ray_directions=candidate_ray_list)

            for hit_iter in range(len(object_hit_info[0])):
                ray = tuple(candidate_ray_list[object_hit_info[1][hit_iter]])
                occ_distance = position.distanceTo(Vector(*object_hit_info[0][hit_iter,:]))

                if occ_distance <= target_distance:
                    # The ray is occluded
                    return False

        return True
    else:
        raise NotImplementedError("Cannot check if " + str(target) + " of type " + type(target) + " can be seen.")

def enableDynamicProxyFor(obj):
    object.__setattr__(obj, '_dynamicProxy', obj._copyWith())

def setDynamicProxyFor(obj, proxy):
    object.__setattr__(obj, '_dynamicProxy', proxy)

def disableDynamicProxyFor(obj):
    object.__setattr__(obj, '_dynamicProxy', obj)

## 2D Compatibility Classes

class Point2D(Point):
    """A 2D version of `Point`, used for backwards compatibility with Scenic 2.0"""
    _scenic_properties = {}

    @cached_property
    def visibleRegion(self):
        """The :term:`visible region` of this 2D point.

        The visible region of a `Point` is a disc centered at its ``position`` with
        radius ``visibleDistance``.
        """
        return CircularRegion(self.position, self.visibleDistance)

class OrientedPoint2D(Point2D, OrientedPoint):
    """A 2D version of `OrientedPoint`, used for backwards compatibility with Scenic 2.0"""
    _scenic_properties = {}

    def __init_subclass__(cls):
        if cls.__dict__.get('_props_transformed', False):
            # Can get here when cls is unpickled (the transformed version was pickled)
            pass
        else:
            cls._props_transformed = True
            # Raise error if parentOrientation already defined
            if 'parentOrientation' in cls._scenic_properties:
                raise RuntimeError('this scenario cannot be run with the --2d flag (the '
                                   f'{cls.__name__} class defines "parentOrientation")')

            # Map certain properties to their 3D analog
            if 'heading' in cls._scenic_properties:
                cls._scenic_properties['parentOrientation'] = cls._scenic_properties['heading']
                del cls._scenic_properties['heading']

        super().__init_subclass__()

    @classmethod
    def _prepareSpecifiers(cls, specifiers):
        # Map certain specifiers to their 3D analog
        newspecs = []
        for spec in specifiers:
            # Map "with heading x" to "facing x"
            if spec.name == "With" and tuple(spec.priorities.keys()) == ('heading',):
                import scenic.syntax.veneer as veneer
                newspecs.append(veneer.Facing(spec.value['heading']))
            else:
                newspecs.append(spec)
        return newspecs

    @cached_property
    def visibleRegion(self):
        """The :term:`visible region` of this 2D oriented point.

        The visible region of an `OrientedPoint` is a sector of the disc centered at its
        ``position`` with radius ``visibleDistance``, oriented along ``heading`` and
        subtending an angle of ``viewAngle``.
        """
        return SectorRegion(self.position, self.visibleDistance,
                            self.heading, self.viewAngle)

class Object2D(OrientedPoint2D, Object):
    """A 2D version of `Object`, used for backwards compatibility with Scenic 2.0"""
    _scenic_properties = {
        'baseOffset': (0,0,0),
        'contactTolerance': 0,
        'requireVisible': True,
        'occluding': False,
    }

    @cached_property
    def visibleRegion(self):
        """The :term:`visible region` of this 2D object.

        The visible region of a 2D `Object` is a circular sector as for `OrientedPoint`,
        except that the base of the sector may be offset from ``position`` by the
        ``cameraOffset`` property (to allow modeling cameras which are not located at the
        center of the object).
        """
        camera = self.position.offsetRotated(self.heading, self.cameraOffset)
        return SectorRegion(camera, self.visibleDistance, self.heading, self.viewAngle)
