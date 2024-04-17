
"""Features, feature spaces, and domains.

.. testsetup::
    from verifai.features import *
"""

import math
import random
import itertools
import functools
from collections import OrderedDict, namedtuple

import numpy as np

from verifai.utils.utils import RejectionException, cached_property

### Domains

class Domain:
    """Abstract class of domains"""

    def distance(self, pointA, pointB):
        raise NotImplementedError('domain distance metric not implemented')

    def uniformPoint(self):
        """Sample a uniformly random point in this Domain"""
        raise NotImplementedError('domain uniform sampling not implemented')

    def flatten(self, point):
        """Flatten a point in this Domain to a tuple of coordinates.

        Useful for analyses that do not understand the internal structure of
        Domains. This representation of a point is also hashable, and so can be
        put into sets and dicts.
        """
        coords = []
        self.flattenOnto(point, coords)
        return tuple(coords)

    def flattenOnto(self, point, targetList):
        """Flatten a point onto the end of the given list."""
        raise NotImplementedError('domain flattening not implemented')

    @cached_property
    def flattenedDimension(self):
        """Dimension of the vector returned by flatten."""
        return NotImplementedError('domain flattening not implemented')

    def meaningOfFlatCoordinate(self, index, pointName='point'):
        """Meaning of a coordinate of a flattened point in this Domain.

        If pointName is the name of a variable storing a point in the Domain,
        then this function returns an expression extracting from that variable
        the value which would be stored in the given coordinate index when the
        point is flattened. For example:

        >>> struct = Struct({'a': Real(), 'b': Real()})
        >>> point = struct.makePoint(a=4, b=3)
        >>> struct.flatten(point)
        (4.0, 3.0)
        >>> struct.meaningOfFlatCoordinate(1)
        'point.b'
        >>> eval(struct.meaningOfFlatCoordinate(1))
        3
        """
        return NotImplementedError('domain flattening not implemented')

    def pandasIndexForFlatCoordinate(self, index):
        """Like meaningOfFlatCoordinate, but giving a MultiIndex for pandas."""
        return NotImplementedError('domain flattening not implemented')

    def coordinateIsNumerical(self, index):
        """Whether the value of a coordinate is intrinsically numerical.

        In particular, whether it makes sense to use the Euclidean distance
        between different values of the coordinate. This would not be the case
        for Domains whose points are strings, for example, even if those are
        converted to numbers for the purpose of flattening.
        """
        return NotImplementedError('domain flattening not implemented')

    def numericizeCoordinate(self, coord):
        """Make a coordinate numeric. For internal use."""
        return coord

    def denumericizeCoordinate(self, coord):
        """Reconstitute a coordinate's original value. For internal use."""
        return coord

    def unflatten(self, coords):
        """Unflatten a tuple of coordinates to a point in this Domain."""
        iterator = iter(coords)
        try:
            return self.unflattenIterator(iterator)
        except StopIteration as e:
            raise RuntimeError('malformed flattened point') from e

    def unflattenIterator(self, coords):
        """Unflatten an iterator of coordinates to a point in this Domain."""
        raise NotImplementedError('domain unflattening not implemented')

    def standardize(self, point):
        """Map the point into a hyperbox, preserving measure.

        If the Domain is continuous, this should map into a unit hyperbox.
        If it is discrete, it should map into a discrete hyperbox.
        Which (if either) of these is the case can be determined by calling
        standardizedDimension and standardizedIntervals: for primitive Domains,
        at least one will return the 'not supported' value."""
        coords = []
        self.standardizeOnto(point, coords)
        return tuple(coords)

    @cached_property
    def isStandardizable(self):
        """Whether points in this Domain can be standardized as above."""
        return self.standardizedDimension >= 0 or self.standardizedIntervals

    def standardizeOnto(self, point, targetList):
        """Standardize a point onto the end of the given list."""
        raise RuntimeError(
            f'Domain {self.__class__.__name__} does not support standardize')

    @cached_property
    def standardizedDimension(self):
        """Dimension of the vector returned by standardize.

        Returns -1 if continuous standardization is not supported."""
        return -1

    @cached_property
    def standardizedIntervals(self):
        """Intervals of the DiscreteBox mapped into by standardize.

        Returns () if discrete standardization is not supported."""
        return ()

    def unstandardize(self, coords):
        """Unstandardize a tuple of coordinates to a point in this Domain."""
        iterator = iter(coords)
        try:
            return self.unstandardizeIterator(iterator)
        except StopIteration as e:
            raise RuntimeError('malformed standardized point') from e

    def unstandardizeIterator(self, coords):
        """Unstandardize an iterator of coords to a point in this Domain."""
        raise RuntimeError(
            f'Domain {self.__class__.__name__} does not support standardize')

    @cached_property
    def requiresRejection(self):
        """Whether sampling from this Domain requires rejection sampling."""
        return False

    def partition(self, predicate):
        """Split this Domain into parts satisfying/falsifying the predicate."""
        if predicate(self):
            return self, None
        else:
            return None, self

    def rejoinPoints(self, *components):
        """Join points from the partitioned components of a Domain."""
        assert sum(int(component is not None) for component in components) == 1
        for component in components:
            if component is not None:
                return component

    @cached_property
    def pointsAreScalars(self):
        """Whether points in this Domain are scalars."""
        return False

## Primitive domains

class Constant(Domain):
    """Domain consisting of a single value"""

    def __init__(self, value):
        try:
            hash(value)
        except TypeError as e:
            raise RuntimeError('value for Constant domain '
                               f'is not hashable: {value}') from e
        self.value = value

    def distance(self, pointA, pointB):
        return 0

    def uniformPoint(self):
        return self.value

    def flattenOnto(self, point, targetList):
        pass    # do nothing: no point in encoding the constant value

    @cached_property
    def flattenedDimension(self):
        return 0

    def meaningOfFlatCoordinate(self, index, pointName='point'):
        raise RuntimeError('Constant domain has no coordinates')

    def pandasIndexForFlatCoordinate(self, index):
        raise RuntimeError('Constant domain has no coordinates')

    def coordinateIsNumerical(self, index):
        raise RuntimeError('Constant domain has no coordinates')

    def unflattenIterator(self, coords):
        return self.value

    def __eq__(self, other):
        return (type(other) is Constant and self.value == other.value)

    def __hash__(self):
        return hash(self.value)

    def __iter__(self):
        yield self.value

    def __repr__(self):
        return f'Constant({self.value})'

class Categorical(Domain):
    """Domain consisting of a finite set of values"""

    def __init__(self, *values):
        self.values = tuple(values)
        if len(self.values) == 0:
            raise RuntimeError('Categorical must contain at least one value')
        try:
            self.index = { val: ind for ind, val in enumerate(self.values) }
        except TypeError as e:
            raise RuntimeError('points in a Domain must be hashable') from e
        if len(self.index) != len(self.values):
            raise RuntimeError('duplicate values in Categorical domain')

    def distance(self, pointA, pointB):
        return 0 if pointA == pointB else 1

    def uniformPoint(self):
        return random.choice(self.values)

    def flattenOnto(self, point, targetList):
        targetList.append(self.numericizeCoordinate(point))

    @cached_property
    def flattenedDimension(self):
        return 1

    def meaningOfFlatCoordinate(self, index, pointName='point'):
        assert index == 0
        return pointName

    def pandasIndexForFlatCoordinate(self, index):
        assert index == 0
        return ()

    def coordinateIsNumerical(self, index):
        assert index == 0
        return False

    def numericizeCoordinate(self, coord):
        return self.index[coord]

    def denumericizeCoordinate(self, coord):
        return self.values[coord]

    def unflattenIterator(self, coords):
        return self.denumericizeCoordinate(next(coords))

    def standardizeOnto(self, point, targetList):
        targetList.append(self.numericizeCoordinate(point))

    @cached_property
    def standardizedIntervals(self):
        return ((0, len(self.values)-1),)

    def unstandardizeIterator(self, coords):
        return self.denumericizeCoordinate(next(coords))

    @cached_property
    def pointsAreScalars(self):
        return True     # TODO hmm

    def __eq__(self, other):
        if type(other) is not Categorical:
            return False
        return set(self.values) == set(other.values)

    def __hash__(self):
        return hash(self.values)

    def __iter__(self):
        yield from self.values

    def __repr__(self):
        return f'Categorical({self.values})'

class Real(Domain):
    """Domain of real numbers"""

    def distance(self, pointA, pointB):
        return math.abs(pointA - pointB)

    def uniformPoint(self):
        raise RuntimeError('cannot sample uniformly from all real numbers')

    def flattenOnto(self, point, targetList):
        targetList.append(float(point))

    @cached_property
    def flattenedDimension(self):
        return 1

    def meaningOfFlatCoordinate(self, index, pointName='point'):
        assert index == 0
        return pointName

    def pandasIndexForFlatCoordinate(self, index):
        assert index == 0
        return ()

    def coordinateIsNumerical(self, index):
        assert index == 0
        return True

    def unflattenIterator(self, coords):
        return next(coords)

    @cached_property
    def pointsAreScalars(self):
        return True

    def __eq__(self, other):
        return type(other) is Real

    def __hash__(self):
        return hash(Real)

    def __repr__(self):
        return 'Real()'

class Integer(Domain):
    """Domain of integers"""

    def distance(self, pointA, pointB):
        return math.abs(pointA - pointB)

    def uniformPoint(self):
        raise RuntimeError('cannot sample uniformly from all integers')

    def flattenOnto(self, point, targetList):
        targetList.append(int(point))

    @cached_property
    def flattenedDimension(self):
        return 1

    def meaningOfFlatCoordinate(self, index, pointName='point'):
        assert index == 0
        return pointName

    def pandasIndexForFlatCoordinate(self, index):
        assert index == 0
        return ()

    def coordinateIsNumerical(self, index):
        assert index == 0
        return True

    def unflattenIterator(self, coords):
        return next(coords)

    @cached_property
    def pointsAreScalars(self):
        return True

    def __eq__(self, other):
        return type(other) is Integer

    def __hash__(self):
        return hash(Integer)

    def __repr__(self):
        return 'Integer()'

class AbstractBox(Domain):
    """A hyper-box, i.e. a product of intervals"""

    def __init__(self, *intervals):
        """Create an AbstractBox from a sequence of intervals"""
        ints = tuple(intervals)
        dim = len(ints)
        if dim == 0:
            raise RuntimeError('attempted to create zero-dimensional Box')
        newInts = []
        lefts = []
        lengths = []
        for interval in ints:
            endpoints = tuple(interval)
            if len(endpoints) != 2:
                raise RuntimeError('interval for Box must have two endpoints')
            left, right = endpoints
            if left > right:
                left, right = right, left
            newInts.append((left, right))
            lefts.append(left)
            lengths.append(right - left)
        self.intervals = tuple(newInts)
        self.lefts = tuple(lefts)
        self.lengths = tuple(lengths)

    def distance(self, pointA, pointB):
        pa, pb = np.array(pointA), np.array(pointB)
        return np.linalg.norm(pa - pb)

    def flattenOnto(self, point, targetList):
        targetList.extend(point)

    @cached_property
    def flattenedDimension(self):
        return self.dimension

    def meaningOfFlatCoordinate(self, index, pointName='point'):
        assert 0 <= index < self.dimension
        return f'{pointName}[{index}]'

    def pandasIndexForFlatCoordinate(self, index):
        assert 0 <= index < self.dimension
        return (index,)

    def coordinateIsNumerical(self, index):
        assert 0 <= index < self.dimension
        return True

    def unflattenIterator(self, coords):
        return tuple(itertools.islice(coords, self.dimension))

    @cached_property
    def pointsAreScalars(self):
        return self.dimension == 1

    @cached_property
    def dimension(self):
        return len(self.intervals)

    def __eq__(self, other):
        if type(other) is not type(self):
            return NotImplemented
        return self.intervals == other.intervals

    def __hash__(self):
        return hash(self.intervals)

class Box(AbstractBox):
    """A hyper-box over the reals.

    Points in a Box are tuples of floats.
    """

    def uniformPoint(self):
        return tuple(random.uniform(lo, hi) for lo, hi in self.intervals)

    def standardizeOnto(self, point, targetList):
        for coord, left, length in zip(point, self.lefts, self.lengths):
            targetList.append((coord - left) / length)

    @cached_property
    def standardizedDimension(self):
        return len(self.intervals)

    def unstandardizeIterator(self, coords):
        point = tuple(
            left + (coord * length)
            for left, length, coord in zip(self.lefts, self.lengths, coords)
        )
        assert len(point) == self.dimension
        return point

    def __repr__(self):
        args = ', '.join(repr(interval) for interval in self.intervals)
        return f'Box({args})'

class DiscreteBox(AbstractBox):
    """A hyper-box over the integers.

    Points in a DiscreteBox are tuples of ints.
    """

    def __init__(self, *intervals):
        super().__init__(*intervals)
        self.intervals = tuple((math.ceil(left), math.floor(right))
                               for left, right in self.intervals)
        for left, right in self.intervals:
            if left > right:
                raise RuntimeError('DiscreteBox interval contains no integers')

    def uniformPoint(self):
        return tuple(random.randint(lo, hi) for lo, hi in self.intervals)

    def standardizeOnto(self, point, targetList):
        targetList.extend(point)

    @cached_property
    def standardizedIntervals(self):
        return self.intervals

    def unstandardizeIterator(self, coords):
        return tuple(itertools.islice(coords, self.dimension))

    def __iter__(self):
        ranges = (range(left, right+1) for left, right in self.intervals)
        yield from itertools.product(*ranges)

    def __repr__(self):
        args = ', '.join(repr(interval) for interval in self.intervals)
        return f'DiscreteBox({args})'

## Domains constructed from other domains

class Array(Domain):
    """A multidimensional array of elements in a common domain.

    For example, Array(Box((-1, 1)), (10, 5)) represents a 10x5 grid
    of real numbers, each in the interval [-1, 1].

    Points in an Array are nested tuples of elements. For example, a point in
    the Array above would be a tuple of 10 elements, each of which is a tuple
    of 5 elements, each of which is a point in the underlying Box domain.
    """

    def __init__(self, domain, shape):
        self.domain = domain
        self.shape = tuple(map(int, shape))
        if len(self.shape) == 0:
            raise RuntimeError('Array shape must be nonempty')
        elts = 1
        for n in self.shape:
            if n < 0:
                raise RuntimeError('Array dimensions must be nonnegative')
            elif elts == 0:
                raise RuntimeError('only the last dimension of an Array can be zero')
            elts *= n
        self.numElements = elts

    def pointWithElements(self, it):
        """Build a point in this domain from an iterable of elements.

        This is similar to numpy.reshape, building a multidimensional array
        from a flat list of elements. For example:

        >>> elts = [1, 2, 3, 4, 5, 6]
        >>> array = Array(Real(), (2, 3))
        >>> array.pointWithElements(elts)
        ((1, 2, 3), (4, 5, 6))
        >>> array = Array(Real(), (3, 2))
        >>> array.pointWithElements(elts)
        ((1, 2), (3, 4), (5, 6))
        """
        it = iter(it)
        shape = self.shape
        dim = len(shape)
        def makeLevel(i):
            if i == dim:    # bottom level is a point in the underlying domain
                return next(it)
            else:   # upper levels are tuples of sublevels
                j = i + 1
                return tuple(makeLevel(j) for k in range(shape[i]))
        return makeLevel(0)

    def elementsOfPoint(self, point):
        """Return an iterator over the elements of a point in this domain.

        This is similar to numpy.flatten, turning a multidimensional array into
        a flat list of elements. For example:

        >>> array = Array(Real(), (3, 2))
        >>> list(array.elementsOfPoint(((1, 2), (3, 4), (5, 6))))
        [1, 2, 3, 4, 5, 6]
        """
        dim = len(self.shape)
        def iterateLevel(i, level):
            if i == dim:    # at bottom, yield the next element
                yield level
            else:   # recursively yield elements of each sublevel
                j = i + 1
                for sublevel in level:
                    yield from iterateLevel(j, sublevel)
        yield from iterateLevel(0, point)

    def uniformPoint(self):
        return self.pointWithElements(iter(self.domain.uniformPoint, None))

    def flattenOnto(self, point, targetList):
        for element in self.elementsOfPoint(point):
            self.domain.flattenOnto(element, targetList)

    @cached_property
    def flattenedDimension(self):
        return self.numElements * self.domain.flattenedDimension

    def meaningOfFlatCoordinate(self, index, pointName='point'):
        assert 0 <= index < self.flattenedDimension
        ourIndex = index // self.domain.flattenedDimension
        subIndex = index % self.domain.flattenedDimension
        indices = []
        for levelSize in reversed(self.shape):
            levelIndex = ourIndex % levelSize
            indices.insert(0, f'[{levelIndex}]')
            ourIndex = ourIndex // levelSize
        meaning = pointName + ''.join(indices)
        return self.domain.meaningOfFlatCoordinate(subIndex, pointName=meaning)

    def pandasIndexForFlatCoordinate(self, index):
        assert 0 <= index < self.flattenedDimension
        ourIndex = index // self.domain.flattenedDimension
        subIndex = index % self.domain.flattenedDimension
        indices = []
        for levelSize in reversed(self.shape):
            levelIndex = ourIndex % levelSize
            indices.insert(0, levelIndex)
            ourIndex = ourIndex // levelSize
        subPandasIndex = self.domain.pandasIndexForFlatCoordinate(subIndex)
        return tuple(indices) + subPandasIndex

    def coordinateIsNumerical(self, index):
        assert 0 <= index < self.flattenedDimension
        subIndex = index % self.domain.flattenedDimension
        return self.domain.coordinateIsNumerical(subIndex)

    def unflattenIterator(self, coords):
        it = iter(lambda: self.domain.unflattenIterator(coords), None)
        return self.pointWithElements(it)

    def standardizeOnto(self, point, targetList):
        for element in self.elementsOfPoint(point):
            self.domain.standardizeOnto(element, targetList)

    @cached_property
    def standardizedDimension(self):
        sd = self.domain.standardizedDimension
        return self.numElements * sd if sd >= 0 else -1

    @cached_property
    def standardizedIntervals(self):
        return self.numElements * self.domain.standardizedIntervals

    def unstandardizeIterator(self, coords):
        it = iter(lambda: self.domain.unstandardizeIterator(coords), None)
        return self.pointWithElements(it)

    def partition(self, predicate):
        if self.numElements == 0:   # length-0 arrays count as leaves
            return super().partition(predicate)
        left, right = self.domain.partition(predicate)
        left = Array(left, self.shape) if left else None
        right = Array(right, self.shape) if right else None
        return left, right

    def rejoinPoints(self, *components):
        values = []
        assert any(component is not None for component in components)
        its = (itertools.repeat(None)
               if comp is None
               else self.elementsOfPoint(comp)
               for comp in components)
        for elements in zip(*its):
            values.append(self.domain.rejoinPoints(*elements))
        return self.pointWithElements(values)

    @cached_property
    def requiresRejection(self):
        return self.domain.requiresRejection

    @cached_property
    def pointsAreScalars(self):
        return self.shape == (1,)

    def __eq__(self, other):
        if not isinstance(other, Array):
            return NotImplemented
        return (self.domain == other.domain and self.shape == other.shape)

    def __hash__(self):
        return hash((self.domain, self.shape))

    def __iter__(self):
        for values in itertools.product(self.domain, repeat=self.numElements):
            yield self.pointWithElements(values)

    def __repr__(self):
        return f'Array({self.domain}, {self.shape})'

class ScalarArray(Array):
    """An array whose elements are integers or reals.

    This is a specialized implementation of Array optimized for large arrays of
    scalars like images.
    """

    def __init__(self, domain, shape):
        super().__init__(domain, shape)
        if not self.domain.pointsAreScalars:
            raise RuntimeError('ScalarArray element domain is not scalar')

    def flattenOnto(self, point, targetList):
        targetList.extend(np.ravel(point))

    @cached_property
    def flattenedDimension(self):
        return self.numElements

    def unflattenIterator(self, coords):
        values = list(itertools.islice(coords, self.numElements))
        return np.reshape(np.array(values), self.shape)

    def __repr__(self):
        return f'ScalarArray({self.domain}, {self.shape})'

class Struct(Domain):
    """A domain consisting of named sub-domains.

    The order of the sub-domains is arbitrary: two Structs are considered equal
    if they have the same named sub-domains, regardless of order. As the order
    is an implementation detail, accessing the values of sub-domains in points
    sampled from a Struct should be done by name:

    >>> struct = Struct({'a': Box((0, 1)), 'b': Box((2, 3))})
    >>> point = struct.uniformPoint()
    >>> point.b
    (2.20215292046797,)

    Within a given version of VerifAI, the sub-domain order is consistent, so
    that the order of columns in error tables is also consistent.
    """

    def __init__(self, domains):
        self.namedDomains = tuple(sorted(domains.items(), key=lambda i: i[0]))
        self.domainNamed = OrderedDict(self.namedDomains)
        self.domains = tuple(self.domainNamed.values())
        self.makePoint = namedtuple('StructPoint', self.domainNamed.keys())

    def uniformPoint(self):
        return self.makePoint(*(d.uniformPoint() for d in self.domains))

    def flattenOnto(self, point, targetList):
        for subPoint, domain in zip(point, self.domains):
            domain.flattenOnto(subPoint, targetList)

    @cached_property
    def flattenedDimension(self):
        return sum(domain.flattenedDimension for domain in self.domains)

    def meaningOfFlatCoordinate(self, index, pointName='point'):
        assert 0 <= index < self.flattenedDimension
        for name, domain in self.namedDomains:
            if index < domain.flattenedDimension:
                subPoint = f'{pointName}.{name}'
                return domain.meaningOfFlatCoordinate(index,
                                                      pointName=subPoint)
            index -= domain.flattenedDimension

    def pandasIndexForFlatCoordinate(self, index):
        assert 0 <= index < self.flattenedDimension
        for name, domain in self.namedDomains:
            if index < domain.flattenedDimension:
                return (name,) + domain.pandasIndexForFlatCoordinate(index)
            index -= domain.flattenedDimension

    def coordinateIsNumerical(self, index):
        assert 0 <= index < self.flattenedDimension
        for name, domain in self.namedDomains:
            if index < domain.flattenedDimension:
                return domain.coordinateIsNumerical(index)
            index -= domain.flattenedDimension

    def unflattenIterator(self, coords):
        subPts = (domain.unflattenIterator(coords) for domain in self.domains)
        return self.makePoint(*subPts)

    def standardizeOnto(self, point, targetList):
        for subPoint, domain in zip(point, self.domains):
            domain.standardizeOnto(subPoint, targetList)

    @cached_property
    def standardizedDimension(self):
        total = 0
        for domain in self.domains:
            sd = domain.standardizedDimension
            if sd < 0:
                return -1
            else:
                total += sd
        return total

    @cached_property
    def standardizedIntervals(self):
        ints = []
        for domain in self.domains:
            subints = domain.standardizedIntervals
            if subints == ():
                return ()
            else:
                ints.extend(subints)
        return tuple(ints)

    def unstandardizeIterator(self, coords):
        subPts = (dom.unstandardizeIterator(coords) for dom in self.domains)
        return self.makePoint(*subPts)

    def partition(self, predicate):
        leftSubs, rightSubs = {}, {}
        for name, domain in self.namedDomains:
            left, right = domain.partition(predicate)
            if left:
                leftSubs[name] = left
            if right:
                rightSubs[name] = right
        left = Struct(leftSubs) if leftSubs else None
        right = Struct(rightSubs) if rightSubs else None
        return left, right

    def rejoinPoints(self, *components):
        subPoints = (
            domain.rejoinPoints(*(getattr(comp, name, None)
                                  for comp in components))
            for name, domain in self.namedDomains
        )
        return self.makePoint(*subPoints)

    @cached_property
    def requiresRejection(self):
        return any(domain.requiresRejection for domain in self.domains)

    def __iter__(self):
        for values in itertools.product(*self.domains):
            yield self.makePoint(*values)

    def __eq__(self, other):
        if type(other) is not Struct:
            return NotImplemented
        return self.namedDomains == other.namedDomains

    def __hash__(self):
        return hash(self.namedDomains)

    def __repr__(self):
        return f'Struct({self.domainNamed})'

class FilteredDomain(Domain):
    """A domain filtered from another by an arbitrary function."""

    def __init__(self, domain, filterFunction):
        self.domain = domain
        self.filter = filterFunction

    def uniformPoint(self):
        sample = self.domain.uniformPoint()
        if self.filter(sample):
            return sample
        else:
            raise RejectionException

    def flattenOnto(self, point, targetList):
        self.domain.flattenOnto(point, targetList)

    @cached_property
    def flattenedDimension(self):
        return self.domain.flattenedDimension

    def meaningOfFlatCoordinate(self, index, pointName='point'):
        return self.domain.meaningOfFlatCoordinate(index,
                                                   pointName=pointName)

    def pandasIndexForFlatCoordinate(self, index):
        return self.domain.pandasIndexForFlatCoordinate(index)

    def coordinateIsNumerical(self, index):
        return self.domain.coordinateIsNumerical(index)

    def unflattenIterator(self, coords):
        return self.domain.unflattenIterator(coords)

    @cached_property
    def requiresRejection(self):
        return True

    @cached_property
    def pointsAreScalars(self):
        return self.domain.pointsAreScalars

    def __iter__(self):
        return filter(self.filter, self.domain)

    def __eq__(self, other):
        if type(other) is not FilteredDomain:
            return NotImplemented
        return (self.domain == other.domain and self.filter == other.filter)

    def __hash__(self):
        return hash((self.domain, self.filter))

    def __repr__(self):
        return f'FilteredDomain({self.domain}, {self.filter})'

### Features

class Feature:
    """A feature or list of features with unknown length.

    Args:
        domain: a `Domain` object specifying the Feature's possible values;
        distribution (optional): object specifying the distribution of values;
        lengthDomain (`Domain`): if not None, this `Feature` is actually a list
          of features, with possible lengths given by this `Domain`;
        lengthDistribution (optional): distribution over lengths;
        distanceMetric (optional): if not None, custom distance metric.

    .. testcode::

        # Feature consisting of list of 10 cars
        carDomain = Struct({
            'position': Array(Real(), [3]),
            'heading': Box((0, math.pi))
        })
        Feature(Array(carDomain, [10]))

        # Feature consisting of list of 1-10 cars
        Feature(carDomain, lengthDomain=DiscreteBox((1, 10)))
    """
    def __init__(self,
                 domain,
                 distribution=None,
                 lengthDomain=None,
                 lengthDistribution=None,
                 distanceMetric=None):
        self.domain = domain
        self.distribution = distribution
        self.lengthDomain = lengthDomain
        self.lengthDistribution = lengthDistribution
        if lengthDomain is None:
            self.minLength = self.maxLength = None
        else:
            if not isinstance(lengthDomain, DiscreteBox):
                raise RuntimeError(
                    'Feature length domain must be a DiscreteBox')
            if lengthDomain.dimension != 1:
                raise RuntimeError('Feature length domain must be an interval')
            self.minLength, self.maxLength = lengthDomain.intervals[0]
            if self.minLength < 0:
                raise RuntimeError('Feature length domain must be nonnegative')
            if (lengthDistribution is not None
                    and len(lengthDistribution.dependencies) > 0):
                raise RuntimeError(
                    'lengths of feature lists must be primitive distributions')
        self.distanceMetric = distanceMetric

    def distance(self, valueA, valueB):
        if self.distanceMetric is None:
            return self.domain.distance(valueA, valueB)
        else:
            return self.distanceMetric(valueA, valueB)

    @cached_property
    def fixedDomains(self):
        """Return the fixed-length Domains associated with this feature."""
        if not self.lengthDomain:
            return self.domain
        domains = {}
        for length in self.lengthDomain:
            length = length[0]
            domains[length] = Array(self.domain, (length,))
        return domains

    def __repr__(self):
        rep = f'Feature({self.domain}'
        if self.distribution is not None:
            rep += f', distribution={self.distribution}'
        if self.lengthDomain is not None:
            rep += f', lengthDomain={self.lengthDomain}'
        if self.lengthDistribution is not None:
            rep += f', lengthDistribution={self.lengthDistribution}'
        if self.distanceMetric is not None:
            rep += f', distanceMetric={self.distanceMetric}'
        return rep + ')'

### Feature spaces

class FeatureSpace:
    """A space consisting of named features.

    .. testcode::

        FeatureSpace({
            'weather': Feature(DiscreteBox([0, 12])),
            'egoCar': Feature(carDomain),
            'traffic': Feature(Array(carDomain, [4]))
        })
    """

    def __init__(self, features, distanceMetric=None):
        self.namedFeatures = tuple(sorted(features.items(), key=lambda i: i[0]))
        self.featureNamed = OrderedDict(self.namedFeatures)
        self.features = tuple(self.featureNamed.values())
        self.makePoint = namedtuple('SpacePoint', self.featureNamed.keys())
        self.distanceMetric = distanceMetric

    @cached_property
    def domains(self):
        """Return the domain or domains associated with this space.

        Returns a pair consisting of the Domain of all lengths of feature
        lists, plus a dict mapping each (flattened) point in that Domain to the
        corresponding Domain of other features. If the FeatureSpace has no
        feature lists, then returns (None, dom) where dom is the fixed Domain
        of all features.
        """
        fixedDomains = {}
        lengthDomains = {}
        variableDomains = {}
        for name, feature in self.namedFeatures:
            if feature.lengthDomain:
                lengthDomains[name] = feature.lengthDomain
                variableDomains[name] = feature.fixedDomains
            else:
                fixedDomains[name] = feature.domain
        if len(lengthDomains) == 0:
            return (None, Struct(fixedDomains))
        lengthDomain = Struct(lengthDomains)
        domainForLengths = {}
        for lengths in lengthDomain:
            domains = dict(fixedDomains)
            # add domains for all feature lists with nonzero lengths
            for name, length in zip(lengths._fields, lengths):
                vdomains = variableDomains[name]
                length = length[0]
                domains[name] = vdomains[length]
            # combine domains into whole domain for this assignment of lengths
            domainForLengths[lengths] = Struct(domains)
        return (lengthDomain, domainForLengths)

    def distance(self, pointA, pointB):
        distances = tuple(
            feature.distance(subptA, subptB)
            for feature, subptA, subptB in zip(self.features, pointA, pointB)
        )
        if self.distanceMetric is None:
            return np.linalg.norm(distances)
        else:
            return self.distanceMetric(distances)

    def flatten(self, point, fixedDimension=False):
        """Flatten a point in this space. See Domain.flatten.

        If fixedDimension is True, the point is flattened out as if all feature
        lists had their maximum lengths, with None as a placeholder. This means
        that all points in the space will flatten to the same length.
        """
        flattened = []
        for feature, value in zip(self.features, point):
            domain = feature.domain
            if feature.lengthDomain:
                length = len(value)
                flattened.append(length)
                fixedDomain = feature.fixedDomains[length]
                fixedDomain.flattenOnto(value, flattened)
                if fixedDimension:      # add padding to maximum length
                    sizePerElt = domain.flattenedDimension
                    needed = (feature.maxLength - length) * sizePerElt
                    for i in range(needed):
                        flattened.append(None)
            else:
                domain.flattenOnto(value, flattened)
        return tuple(flattened)

    @cached_property
    def fixedFlattenedDimension(self):
        """Length of vector returned by flatten with fixedDimension=True.

        Also an upper bound on the length of the vector returned by flatten
        by default, when fixedDimension=False."""
        dim = 0
        for feature in self.features:
            domain = feature.domain
            if feature.lengthDomain:
                dim += 1    # dimension storing length of the feature list
                dim += feature.maxLength * domain.flattenedDimension
            else:
                dim += domain.flattenedDimension
        return dim

    def meaningOfFlatCoordinate(self, index, pointName='point'):
        """Meaning of a coordinate of a flattened point in this space.

        See the corresponding function of Domain. Works only for points
        flattened with fixedDimension=True, since otherwise a given index can
        have different meaning depending on the lengths of feature lists.
        """
        assert 0 <= index < self.fixedFlattenedDimension
        for name, feature in self.namedFeatures:
            domain = feature.domain
            if feature.lengthDomain:
                if index == 0:
                    return f'len({pointName}.{name})'
                else:
                    index -= 1
                    elem = index // domain.flattenedDimension
                    if elem < feature.maxLength:
                        subIndex = index % domain.flattenedDimension
                        subPoint = f'{pointName}.{name}[{elem}]'
                        return domain.meaningOfFlatCoordinate(subIndex,
                            pointName=subPoint)
                    index -= feature.maxLength * domain.flattenedDimension
            else:
                if index < domain.flattenedDimension:
                    subPoint = f'{pointName}.{name}'
                    return domain.meaningOfFlatCoordinate(index,
                                                          pointName=subPoint)
                index -= domain.flattenedDimension
        raise RuntimeError('impossible index arithmetic')

    def pandasIndexForFlatCoordinate(self, index):
        """Pandas index of a coordinate of a flattened point in this space.

        See meaningOfFlatCoordinate, and Domain.pandasIndexForFlatCoordinate.
        """
        assert 0 <= index < self.fixedFlattenedDimension
        for name, feature in self.namedFeatures:
            domain = feature.domain
            if feature.lengthDomain:
                if index == 0:
                    return (name, 'length')
                else:
                    index -= 1
                    elem = index // domain.flattenedDimension
                    if elem < feature.maxLength:
                        subIndex = index % domain.flattenedDimension
                        panda = domain.pandasIndexForFlatCoordinate(subIndex)
                        return (name, elem) + panda
                    index -= feature.maxLength * domain.flattenedDimension
            else:
                if index < domain.flattenedDimension:
                    panda = domain.pandasIndexForFlatCoordinate(index)
                    return (name,) + panda
                index -= domain.flattenedDimension
        raise RuntimeError('impossible index arithmetic')

    def coordinateIsNumerical(self, index):
        """Whether the value of a coordinate is intrinsically numerical.

        See meaningOfFlatCoordinate, and Domain.coordinateIsNumerical.
        """
        assert 0 <= index < self.fixedFlattenedDimension
        for name, feature in self.namedFeatures:
            domain = feature.domain
            if feature.lengthDomain:
                if index == 0:
                    return True
                else:
                    index -= 1
                    elem = index // domain.flattenedDimension
                    if elem < feature.maxLength:
                        subIndex = index % domain.flattenedDimension
                        return domain.coordinateIsNumerical(subIndex)
                    index -= feature.maxLength * domain.flattenedDimension
            else:
                if index < domain.flattenedDimension:
                    return domain.coordinateIsNumerical(index)
                index -= domain.flattenedDimension
        raise RuntimeError('impossible index arithmetic')

    def unflatten(self, coords, fixedDimension=False):
        """Unflatten a tuple of coordinates to a point in this space."""
        values = []
        iterator = iter(coords)
        for feature in self.features:
            domain = feature.domain
            if feature.lengthDomain:
                length = next(iterator)
                fixedDomain = feature.fixedDomains[length]
                values.append(fixedDomain.unflattenIterator(iterator))
                if fixedDimension:      # consume padding
                    sizePerElt = domain.flattenedDimension
                    needed = (feature.maxLength - length) * sizePerElt
                    for i in range(needed):
                        next(iterator)
            else:
                values.append(domain.unflattenIterator(iterator))
        return self.makePoint(*values)

    def __repr__(self):
        rep = f'FeatureSpace({self.featureNamed}'
        if self.distanceMetric is not None:
            rep += f', distanceMetric={self.distanceMetric}'
        return rep + ')'
