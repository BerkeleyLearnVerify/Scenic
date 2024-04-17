
import pytest
from dotmap import DotMap

from verifai.samplers.scenic_sampler import ScenicSampler, scenicMajorVersion
from verifai.scenic_server import ScenicServer
from verifai.falsifier import generic_falsifier
from verifai.monitor import specification_monitor
from tests.utils import sampleWithFeedback, checkSaveRestore

## Basic

def test_objects(new_Object):
    sampler = ScenicSampler.fromScenicCode(
        f'ego = {new_Object} at 4 @ 9',
        maxIterations=1
    )
    sample = sampler.nextSample()
    objects = sample.objects
    assert len(objects) == 1
    pos = objects.object0.position
    assert type(pos) is tuple
    assert pos == (4, 9, 0) if scenicMajorVersion >= 3 else (4, 9)

def test_params(new_Object):
    sampler = ScenicSampler.fromScenicCode(
        'param x = Range(3, 5)\n'
        f'ego = {new_Object}',
        maxIterations=1
    )
    sample = sampler.nextSample()
    x = sample.params.x
    assert type(x) is float
    assert 3 <= x <= 5

def test_quoted_param(new_Object):
    sampler = ScenicSampler.fromScenicCode(
        'param "x/y" = Range(3, 5)\n'
        f'ego = {new_Object}',
        maxIterations=1
    )
    sample = sampler.nextSample()
    v = sampler.paramDictForSample(sample)['x/y']
    assert type(v) is float
    assert 3 <= v <= 5

def test_lists(new_Object):
    sampler = ScenicSampler.fromScenicCode(
        f'ego = {new_Object} with foo [1, -1, 3.3]',
        maxIterations=1
    )
    sample = sampler.nextSample()
    foo = sample.objects.object0.foo
    assert type(foo) is tuple
    assert foo == pytest.approx((1, -1, 3.3))

def test_save_restore(new_Object, tmpdir):
    sampler = ScenicSampler.fromScenicCode(
        f'ego = {new_Object} at Range(-1, 1) @ 0',
        maxIterations=1
    )
    checkSaveRestore(sampler, tmpdir)

def test_object_order(new_Object):
    sampler = ScenicSampler.fromScenicCode(
        f'ego = {new_Object} at 0 @ 0\n'
        'for i in range(1, 11):\n'
        f'    {new_Object} at 2*i @ 0',
        maxIterations=1
    )
    sample = sampler.nextSample()
    objects = sample.objects
    assert len(objects) == 11
    for i in range(len(objects)):
        name = ScenicSampler.nameForObject(i)
        obj = getattr(objects, name)
        assert obj.position[:2] == pytest.approx((2*i, 0))

    flat = sampler.space.flatten(sample)
    unflat = sampler.space.unflatten(flat)
    assert unflat == sample

## Active sampling

def test_active_sampling(new_Object):
    sampler = ScenicSampler.fromScenicCode(
        'from dotmap import DotMap\n'
        'param verifaiSamplerType = "ce"\n'
        'ce_params = DotMap(alpha=0.9)\n'
        'ce_params.cont.buckets = 2\n'
        'param verifaiSamplerParams = ce_params\n'
        f'ego = {new_Object} at VerifaiRange(-1, 1) @ 0',
        maxIterations=1
    )
    def f(sample):
        return -1 if sample.objects.object0.position[0] < 0 else 1
    samples = sampleWithFeedback(sampler, 120, f)
    xs = [sample.objects.object0.position[0] for sample in samples]
    assert all(-1 <= x <= 1 for x in xs)
    assert any(x > 0 for x in xs)
    assert 66 <= sum(x < 0 for x in xs[50:])

def test_active_save_restore(new_Object, tmpdir):
    sampler = ScenicSampler.fromScenicCode(
        'param verifaiSamplerType = "halton"\n'
        f'ego = {new_Object} at VerifaiRange(-1, 1) @ 0',
        maxIterations=1
    )
    checkSaveRestore(sampler, tmpdir)

## Webots

def runSampler(sampler):
    for i in range(3):
        sample = sampler.nextSample()
        print(f'Sample #{i}:')
        print(sample)

def test_webots_road(pathToLocalFile):
    path = pathToLocalFile('scenic_road.scenic')
    sampler = ScenicSampler.fromScenario(path, mode2D=True)
    runSampler(sampler)

## Driving domain

def test_driving(pathToLocalFile):
    path = pathToLocalFile('scenic_driving.scenic')
    sampler = ScenicSampler.fromScenario(path, mode2D=True)
    runSampler(sampler)

## Dynamic scenarios

basic_scenario = """\
param verifaiSamplerType = 'grid'
param render = False
model scenic.simulators.newtonian.model
ego = {new_Object} with velocity (0, VerifaiOptions([0, 10]))
{new_Object} at (10, 0), with velocity (-10, 10)
terminate after 1 seconds
"""

def test_dynamic(new_Object):
    sampler = ScenicSampler.fromScenicCode(basic_scenario.format(new_Object=new_Object))

    class distance_spec(specification_monitor):
        def __init__(self):
            def specification(simulation):
                egoPos, otherPos = simulation.result.trajectory[-1]
                dist = egoPos.distanceTo(otherPos)
                return dist - 1
            super().__init__(specification)

    falsifier = generic_falsifier(
        sampler=sampler,
        monitor=distance_spec(),
        falsifier_params=DotMap(n_iters=2, save_safe_table=True),
        server_class=ScenicServer
    )
    falsifier.run_falsifier()
    assert len(falsifier.error_table.table) == 1
    assert len(falsifier.safe_table.table) == 1

def test_driving_dynamic(pathToLocalFile):
    path = pathToLocalFile('scenic_driving.scenic')
    sampler = ScenicSampler.fromScenario(
        path,
        model='scenic.simulators.newtonian.driving_model',
        params=dict(render=False),
        mode2D=True,
    )
    falsifier_params = DotMap(
        n_iters=3,
        save_error_table=False,
        save_safe_table=False,
    )
    server_options = DotMap(maxSteps=2, verbosity=3)
    falsifier = generic_falsifier(sampler=sampler,
                                  falsifier_params=falsifier_params,
                                  server_class=ScenicServer,
                                  server_options=server_options)
    falsifier.run_falsifier()
