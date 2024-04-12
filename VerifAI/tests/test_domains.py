import math
import pytest

import numpy as np

from verifai.features import *

### Utilities

def checkFlattening(domain, point, expectedLength=None, coordsAreNumeric=None):
    flat = domain.flatten(point)
    assert type(flat) is tuple
    assert len(flat) == domain.flattenedDimension
    if expectedLength is not None:
        assert len(flat) == expectedLength
    unflat = domain.unflatten(flat)
    assert point == unflat
    for index, value in enumerate(flat):
        meaning = domain.meaningOfFlatCoordinate(index, pointName='point')
        extractedValue = eval(meaning)
        assert extractedValue == domain.denumericizeCoordinate(value)
        assert domain.numericizeCoordinate(extractedValue) == value
        if coordsAreNumeric is not None:
            assert domain.coordinateIsNumerical(index) == coordsAreNumeric

### Constant and Categorical

def test_constant_sampling():
    dom = Constant(12.4)
    for i in range(10):
        point = dom.uniformPoint()
        assert point == 12.4

def test_constant_enumeration():
    dom = Constant(-7)
    assert set(dom) == { -7 }

def test_constant_flatten():
    dom = Constant(3.14)
    point = dom.uniformPoint()
    checkFlattening(dom, point, expectedLength=0)

def test_constant_equality():
    dom1 = Constant(4)
    dom2 = Constant(4)
    assert set([dom1]) == set([dom2])

def test_categorical_empty():
    with pytest.raises(RuntimeError):
        Categorical()

def test_categorical_duplicate():
    with pytest.raises(RuntimeError):
        Categorical(1, 2, 1)

def test_categorical_sampling_form():
    vals = (1, 4, 9, 16, 25, 36)
    cat = Categorical(*vals)
    for i in range(10):
        point = cat.uniformPoint()
        assert type(point) is int
        assert point in vals

def test_categorical_sampling_random():
    cat = Categorical(-1.2, 1.7)
    points = tuple(cat.uniformPoint() for i in range(100))
    assert any(point > 0 for point in points)
    assert any(point < 0 for point in points)

def test_categorical_enumeration():
    vals = (1, 4, 9, 16, 25, 36)
    cat = Categorical(*vals)
    assert set(cat) == set(vals)

def test_categorical_flatten():
    vals = (1, 4, 9, 16, 25, 36)
    cat = Categorical(*vals)
    point = cat.uniformPoint()
    checkFlattening(cat, point, expectedLength=1, coordsAreNumeric=False)
    assert cat.pandasIndexForFlatCoordinate(0) == ()

def test_categorical_standardize():
    vals = (1, 4, 9, 16, 25, 36)
    cat = Categorical(*vals)
    assert cat.standardizedDimension == -1
    maxIndex = len(vals) - 1
    assert cat.standardizedIntervals == ((0, maxIndex),)
    for point in cat:
        stand = cat.standardize(point)
        assert type(stand) is tuple
        assert len(stand) == 1
        assert 0 <= stand[0] <= maxIndex
        unstand = cat.unstandardize(stand)
        assert point == unstand

def test_categorical_equality():
    dom1 = Categorical(4, 'blob')
    dom2 = Categorical(4, 'blob')
    assert set([dom1]) == set([dom2])

### Box and DiscreteBox

def test_box_zerodim():
    with pytest.raises(RuntimeError):
        Box()

def test_box_bad_interval():
    with pytest.raises(RuntimeError):
        Box((1, 2), (1, 4, 9))

def test_box_sampling_form():
    def check(box):
        for i in range(10):
            point = box.uniformPoint()
            assert type(point) is tuple
            assert len(point) == 2
            for coord in point:
                assert 0 <= coord <= 2
    check(Box((0, 2), (2, 0)))
    check(DiscreteBox((0, 2), (0, 2)))

def test_box_sampling_random():
    def check(box):
        points = tuple(box.uniformPoint()[0] for i in range(100))
        assert any(point > 5 for point in points)
        assert any(point < 5 for point in points)
    check(Box((0, 10)))
    check(DiscreteBox((0, 10)))

def test_box_enumeration():
    box = DiscreteBox((0, 3), (-1, 1))
    assert set(box) == {
        (0, -1), (0, 0), (0, 1),
        (1, -1), (1, 0), (1, 1),
        (2, -1), (2, 0), (2, 1),
        (3, -1), (3, 0), (3, 1)
    }

def test_box_flatten():
    box = Box((0, 2), (0, 2), (0, 2))
    point = box.uniformPoint()
    checkFlattening(box, point, expectedLength=3, coordsAreNumeric=True)
    for i in range(3):
        assert box.pandasIndexForFlatCoordinate(i) == (i,)

def test_box_standardize():
    box = Box((0, 1), (0, 5), (-3, 3))
    assert box.standardizedDimension == 3
    assert box.standardizedIntervals == ()
    for i in range(10):
        point = box.uniformPoint()
        stand = box.standardize(point)
        assert type(stand) is tuple
        assert len(stand) == 3
        for coord in stand:
            assert 0 <= coord <= 1
        unstand = box.unstandardize(stand)
        assert point == unstand

def test_box_equality():
    dom1 = Box((-3, 5))
    dom2 = Box((-3, 5))
    assert set([dom1]) == set([dom2])

def test_discrete_box_standardize():
    intervals = ((0, 1), (0, 5), (-3, 3))
    box = DiscreteBox(*intervals)
    assert box.standardizedDimension == -1
    assert box.standardizedIntervals == intervals
    for point in box:
        stand = box.standardize(point)
        assert type(stand) is tuple
        assert len(stand) == len(intervals)
        for coord, interval in zip(stand, intervals):
            left, right = interval
            assert left <= coord <= right
        unstand = box.unstandardize(stand)
        assert point == unstand

def test_discrete_box_equality():
    dom1 = DiscreteBox((-3, 5))
    dom2 = DiscreteBox((-3, 5))
    assert set([dom1]) == set([dom2])

### Arrays

def test_array_zerodim():
    with pytest.raises(RuntimeError):
        Array(Box((0, 1)), ())

def test_array_negative_dimension():
    with pytest.raises(RuntimeError):
        Array(Box((0, 1)), (1, 2, -5))

def test_array_sampling_form():
    box = Box((0, 2))
    shape = (3, 2)
    array = Array(box, shape)
    for i in range(10):
        point = array.uniformPoint()
        assert type(point) is tuple
        assert len(point) == 3
        for level in point:
            assert type(level) is tuple
            assert len(level) == 2
        for element in array.elementsOfPoint(point):
            assert 0 <= element[0] <= 2

def test_array_sampling_random():
    box = Box((-1, 1))
    array = Array(box, (100,))
    point = array.uniformPoint()
    assert any(element[0] > 0 for element in point)
    assert any(element[0] < 0 for element in point)

def test_array_enumeration():
    box = DiscreteBox((0, 1))
    array = Array(box, (2, 2))
    assert set(array) == {
        (((0,), (0,)), ((0,), (0,))),
        (((0,), (0,)), ((0,), (1,))),
        (((0,), (0,)), ((1,), (0,))),
        (((0,), (0,)), ((1,), (1,))),
        (((0,), (1,)), ((0,), (0,))),
        (((0,), (1,)), ((0,), (1,))),
        (((0,), (1,)), ((1,), (0,))),
        (((0,), (1,)), ((1,), (1,))),
        (((1,), (0,)), ((0,), (0,))),
        (((1,), (0,)), ((0,), (1,))),
        (((1,), (0,)), ((1,), (0,))),
        (((1,), (0,)), ((1,), (1,))),
        (((1,), (1,)), ((0,), (0,))),
        (((1,), (1,)), ((0,), (1,))),
        (((1,), (1,)), ((1,), (0,))),
        (((1,), (1,)), ((1,), (1,)))
    }

def test_array_element_iteration():
    array = Array(Real(), (2, 3))
    elts = [1, 2, 3, 4, 5, 6]
    point = array.pointWithElements(elts)
    assert point == ((1, 2, 3), (4, 5, 6))
    array = Array(Real(), (3, 2))
    point = array.pointWithElements(elts)
    assert point == ((1, 2), (3, 4), (5, 6))

def test_array_element_indexing():
    array = Array(Box((0, 3)), (2, 3))
    for i in range(10):
        point = array.uniformPoint()
        assert len(point) == 2
        left, right = point[0], point[1]
        assert len(left) == 3
        assert len(right) == 3
        for element in left:
            assert len(element) == 1
            assert 0 <= element[0] <= 3
        for element in right:
            assert len(element) == 1
            assert 0 <= element[0] <= 3

def test_array_flatten():
    box = Box((-1, 1))
    array = Array(box, (5, 2, 3))
    point = array.uniformPoint()
    checkFlattening(array, point, expectedLength=30, coordsAreNumeric=True)

def test_array_pandas():
    box = Box((-1, 1))
    array = Array(box, (5, 2, 3))
    point = array.uniformPoint()
    flat = array.flatten(point)
    for index, value in enumerate(flat):
        panda = array.pandasIndexForFlatCoordinate(index)
        extracted = point
        for subPanda in panda:
            extracted = extracted[subPanda]
        assert extracted == value

def test_array_standardize():
    box = Box((-1, 1))
    array = Array(box, (5, 2, 3))
    assert array.standardizedDimension == 30
    assert array.standardizedIntervals == ()
    for i in range(10):
        point = array.uniformPoint()
        stand = array.standardize(point)
        assert type(stand) is tuple
        assert len(stand) == 30
        for coord in stand:
            assert -1 <= coord <= 1
        unstand = array.unstandardize(stand)
        assert point == unstand

def test_array_discrete_standardize():
    interval = (-1, 1)
    box = DiscreteBox(interval)
    array = Array(box, (5, 2, 3))
    assert array.standardizedDimension == -1
    assert array.standardizedIntervals == tuple(interval for i in range(30))
    for i in range(10):
        point = array.uniformPoint()
        stand = array.standardize(point)
        assert type(stand) is tuple
        assert len(stand) == 30
        for coord in stand:
            assert -1 <= coord <= 1
        unstand = array.unstandardize(stand)
        assert point == unstand

def test_array_equality():
    dom1 = Array(Box((-3, 5)), (6, 2))
    dom2 = Array(Box((-3, 5)), (6, 2))
    assert set([dom1]) == set([dom2])

def test_array_of_arrays():
    box = Box((-1, 1))
    array1 = Array(box, (3,))
    array2 = Array(array1, (4,))
    point = array2.uniformPoint()
    assert type(point) is tuple
    assert len(point) == 4
    for element in point:
        assert type(element) is tuple
        assert len(element) == 3
        for element2 in element:
            assert type(element2) is tuple
            assert len(element2) == 1
            assert -1 <= element2[0] <= 1
    checkFlattening(array2, point, expectedLength=12, coordsAreNumeric=True)

def test_array_empty():
    array = Array(Box((0, 1)), (0,))
    for i in range(10):
        point = array.uniformPoint()
        assert type(point) is tuple
        assert point == ()
    assert tuple(array) == ((),)
    assert array.pointWithElements(()) == ()
    assert tuple(array.elementsOfPoint(())) == ()
    assert array.flattenedDimension == 0
    assert array.flatten(()) == ()
    assert array.unflatten(()) == ()
    assert array.standardizedDimension == 0
    assert array.standardizedIntervals == ()
    assert array.standardize(()) == ()
    assert array.unstandardize(()) == ()

def test_array_empty2():
    array = Array(Box((0, 1)), (3, 0))
    for i in range(10):
        point = array.uniformPoint()
        assert type(point) is tuple
        assert point == ((), (), ())
    assert tuple(array) == (point,)
    assert array.pointWithElements(()) == point
    assert tuple(array.elementsOfPoint(point)) == ()
    assert array.flattenedDimension == 0
    assert array.flatten(point) == ()
    assert array.unflatten(()) == point
    assert array.standardizedDimension == 0
    assert array.standardizedIntervals == ()
    assert array.standardize(point) == ()
    assert array.unstandardize(()) == point

### Structs

def test_struct_sampling_form():
    struct = Struct({
        'a': Box((-1, 1)),
        'b': DiscreteBox((4, 5))
    })
    for i in range(10):
        point = struct.uniformPoint()
        assert type(point) is struct.makePoint
        assert set(point._fields) == { 'a', 'b' }
        assert -1 <= point.a[0] <= 1
        assert 4 <= point.b[0] <= 5

def test_struct_sampling_random():
    box = DiscreteBox((0, 10))
    struct = Struct({ 'a': box, 'b': box })
    points = tuple(struct.flatten(struct.uniformPoint()) for i in range(100))
    assert any(l > r for l, r in points)
    assert any(l < r for l, r in points)

def test_struct_enumeration():
    box = DiscreteBox((0, 2))
    struct = Struct({ 'a': box, 'b': box })
    assert set(struct) == {
        struct.makePoint(**pt) for pt in [
            { 'a': (0,), 'b': (0,) },
            { 'a': (0,), 'b': (1,) },
            { 'a': (0,), 'b': (2,) },
            { 'a': (1,), 'b': (0,) },
            { 'a': (1,), 'b': (1,) },
            { 'a': (1,), 'b': (2,) },
            { 'a': (2,), 'b': (0,) },
            { 'a': (2,), 'b': (1,) },
            { 'a': (2,), 'b': (2,) }
        ]
    }

def test_struct_flatten():
    box = DiscreteBox((0, 2))
    struct = Struct({ 'a': box, 'b': box })
    point = struct.uniformPoint()
    checkFlattening(struct, point, expectedLength=2, coordsAreNumeric=True)

def test_struct_standardize_continuous():
    box = Box((0, 1), (0, 5), (-3, 3))
    struct = Struct({ 'a': box, 'b': box })
    assert struct.standardizedDimension == 6
    assert struct.standardizedIntervals == ()
    for i in range(10):
        point = struct.uniformPoint()
        stand = struct.standardize(point)
        assert type(stand) is tuple
        assert len(stand) == 6
        for coord in stand:
            assert 0 <= coord <= 1
        unstand = struct.unstandardize(stand)
        assert point == unstand

def test_struct_standardize_discrete():
    intervals = ((0, 1), (0, 5), (-3, 3))
    box = DiscreteBox(*intervals)
    struct = Struct({ 'a': box, 'b': box })
    assert struct.standardizedDimension == -1
    assert struct.standardizedIntervals == intervals * 2
    for i in range(10):
        point = struct.uniformPoint()
        stand = struct.standardize(point)
        assert type(stand) is tuple
        assert len(stand) == len(intervals) * 2
        for coord, interval in zip(stand, intervals * 2):
            left, right = interval
            assert left <= coord <= right
        unstand = struct.unstandardize(stand)
        assert point == unstand

def test_struct_standardize_mixed():
    struct = Struct({ 'a': DiscreteBox((0, 1)), 'b': Box((0, 1)) })
    assert struct.standardizedDimension == -1
    assert struct.standardizedIntervals == ()

def test_struct_equality():
    dom1 = Struct({'a': Box((-3, 5)), 'b': Box((0, 1)) })
    dom2 = Struct({'b': Box((0, 1)), 'a': Box((-3, 5)) })
    assert set([dom1]) == set([dom2])

def test_array_of_structs():
    box = Box((-1, 1))
    struct = Struct({ 'a': box, 'b': box })
    array = Array(struct, (2, 5))
    point = array.uniformPoint()
    checkFlattening(array, point, expectedLength=20, coordsAreNumeric=True)

def test_struct_of_arrays():
    box = Box((-1, 1))
    array = Array(box, (5, 2))
    struct = Struct({ 'a': array, 'b': array })
    point = struct.uniformPoint()
    checkFlattening(struct, point, expectedLength=20, coordsAreNumeric=True)

def test_struct_of_structs():
    box = Box((-1, 1))
    struct1 = Struct({ 'a': box, 'b': box })
    struct2 = Struct({ 'c': struct1, 'd': struct1, 'e': box })
    point = struct2.uniformPoint()
    checkFlattening(struct2, point, expectedLength=5, coordsAreNumeric=True)

### Partitioning

def test_partition_primitive():
    dom = Box((1, 2))
    left, right = dom.partition(lambda d: True)
    assert left == dom
    assert right is None
    lpt = left.uniformPoint()
    point = dom.rejoinPoints(lpt, None)
    assert type(point) is tuple
    assert len(point) == 1
    assert 1 <= point[0] <= 2
    left, right = dom.partition(lambda d: False)
    assert left is None
    assert right == dom
    rpt = right.uniformPoint()
    point = dom.rejoinPoints(None, rpt)
    assert type(point) is tuple
    assert len(point) == 1
    assert 1 <= point[0] <= 2

def test_partition_struct():
    box = Box((-1, 1))
    dbox = DiscreteBox((4, 5))
    struct = Struct({ 'a': box, 'b': dbox })
    left, right = struct.partition(lambda d: d.standardizedIntervals != ())
    assert left == Struct({ 'b': dbox })
    assert right == Struct({ 'a': box })
    for i in range(10):
        lpt = left.uniformPoint()
        rpt = right.uniformPoint()
        point = struct.rejoinPoints(lpt, rpt)
        assert type(point) is struct.makePoint
        assert set(point._fields) == { 'a', 'b' }
        assert -1 <= point.a[0] <= 1
        assert 4 <= point.b[0] <= 5

def test_partition_array():
    box = Box((-1, 1))
    array = Array(box, (2, 7, 3))
    left, right = array.partition(lambda d: True)
    assert left == array
    assert right is None
    lpt = left.uniformPoint()
    point = array.rejoinPoints(lpt, None)
    assert type(point) is tuple
    flat = array.flatten(point)
    assert len(flat) == array.numElements
    left, right = array.partition(lambda d: False)
    assert left is None
    assert right == array
    rpt = right.uniformPoint()
    point = array.rejoinPoints(None, rpt)
    assert type(point) is tuple
    flat = array.flatten(point)
    assert len(flat) == array.numElements

def test_partition_array_struct():
    box = Box((-1, 1))
    dbox = DiscreteBox((4, 5))
    struct = Struct({ 'a': box, 'b': dbox })
    array = Array(struct, (4,))
    left, right = array.partition(lambda d: d.standardizedDimension > 0)
    assert left == Array(Struct({ 'a': box }), (4,))
    assert right == Array(Struct({ 'b': dbox }), (4,))
    for i in range(10):
        lpt = left.uniformPoint()
        rpt = right.uniformPoint()
        point = array.rejoinPoints(lpt, rpt)
        assert type(point) is tuple
        assert len(point) == 4
        for element in point:
            assert set(element._fields) == { 'a', 'b' }
            assert -1 <= element.a[0] <= 1
            assert 4 <= element.b[0] <= 5

def test_partition_array_empty():
    struct = Struct({ 'a': Box((-1, 1)), 'b': DiscreteBox((4, 5)) })
    array = Array(struct, (5, 0))
    left, right = array.partition(lambda d: d.standardizedDimension > 0)
    assert left == None
    assert right == array
