from verifai.features import *
from verifai.samplers import FeatureSampler

### FeatureSpaces

def test_fs_flatten():
    space = FeatureSpace({
        'a': Feature(DiscreteBox([0, 12])),
        'b': Feature(Box((0, 1)), lengthDomain=DiscreteBox((0, 2)))
    })
    sampler = FeatureSampler.randomSamplerFor(space)
    for i in range(100):
        point = sampler.nextSample()
        flat = space.flatten(point)
        assert type(flat) is tuple
        assert len(flat) <= space.fixedFlattenedDimension
        unflat = space.unflatten(flat)
        assert point == unflat

def test_fs_flatten_fixed_dimension():
    space = FeatureSpace({
        'a': Feature(DiscreteBox([0, 12])),
        'b': Feature(Box((0, 1)), lengthDomain=DiscreteBox((0, 2)))
    })
    assert space.fixedFlattenedDimension == 4
    sampler = FeatureSampler.randomSamplerFor(space)
    for i in range(100):
        point = sampler.nextSample()
        flat = space.flatten(point, fixedDimension=True)
        assert type(flat) is tuple
        assert len(flat) == 4
        bLen = len(point.b)
        assert eval(space.meaningOfFlatCoordinate(0)) == point.a[0]
        assert flat[1] == bLen
        assert eval(space.meaningOfFlatCoordinate(1)) == bLen
        if bLen < 1:
            assert flat[2] is None
        else:
            assert eval(space.meaningOfFlatCoordinate(2)) == point.b[0][0]
        if bLen < 2:
            assert flat[3] is None
        else:
            assert eval(space.meaningOfFlatCoordinate(3)) == point.b[1][0]
        unflat = space.unflatten(flat, fixedDimension=True)
        assert point == unflat
    assert space.pandasIndexForFlatCoordinate(0) == ('a', 0)
    assert space.pandasIndexForFlatCoordinate(1) == ('b', 'length')
    assert space.pandasIndexForFlatCoordinate(2) == ('b', 0, 0)
    assert space.pandasIndexForFlatCoordinate(3) == ('b', 1, 0)
    assert all(space.coordinateIsNumerical(i) for i in range(4))

def test_fs_flatten_fixed_dimension2():
    cat = Categorical(2, 7, 1, 8)
    space = FeatureSpace({
        'b': Feature(cat, lengthDomain=DiscreteBox((1, 3))),
        'c': Feature(Box([-1, 1], [-3, 3]))
    })
    assert space.fixedFlattenedDimension == 6
    sampler = FeatureSampler.randomSamplerFor(space)
    for i in range(100):
        point = sampler.nextSample()
        flat = space.flatten(point, fixedDimension=True)
        assert type(flat) is tuple
        assert len(flat) == 6
        bLen = len(point.b)
        assert 1 <= bLen <= 3
        assert flat[0] == bLen
        assert eval(space.meaningOfFlatCoordinate(0)) == bLen
        assert eval(space.meaningOfFlatCoordinate(1)) == point.b[0]
        if bLen < 2:
            assert flat[2] is None
        else:
            assert eval(space.meaningOfFlatCoordinate(2)) == point.b[1]
        if bLen < 3:
            assert flat[3] is None
        else:
            assert eval(space.meaningOfFlatCoordinate(3)) == point.b[2]
        unflat = space.unflatten(flat, fixedDimension=True)
        assert point == unflat
    assert space.pandasIndexForFlatCoordinate(0) == ('b', 'length')
    assert space.pandasIndexForFlatCoordinate(1) == ('b', 0)
    assert space.pandasIndexForFlatCoordinate(2) == ('b', 1)
    assert space.pandasIndexForFlatCoordinate(3) == ('b', 2)
    assert space.pandasIndexForFlatCoordinate(4) == ('c', 0)
    assert space.pandasIndexForFlatCoordinate(5) == ('c', 1)
    assert space.coordinateIsNumerical(0)
    assert not any(space.coordinateIsNumerical(i) for i in range(1, 4))
    assert all(space.coordinateIsNumerical(i) for i in range(4, 6))

def test_fs_distance():
    box = Box([0, 10])
    space = FeatureSpace({ 'a': Feature(box), 'b': Feature(box) })
    sampler = FeatureSampler.randomSamplerFor(space)
    pointA = sampler.nextSample()
    pointB = sampler.nextSample()
    assert pointA != pointB
    assert space.distance(pointA, pointA) == 0
    assert space.distance(pointB, pointB) == 0
    assert space.distance(pointA, pointB) != 0
