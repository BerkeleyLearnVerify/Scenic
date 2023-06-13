import pytest

from scenic.core.simulators import DummySimulator, Simulation
from tests.utils import sampleSceneFrom

def test_old_style_simulator():
    with pytest.raises(RuntimeError) as e:
        class MySimulation(Simulation):
            def run(*args, **kwargs):
                assert False
    assert 'old-style simulation API' in str(e)

    with pytest.raises(RuntimeError) as e:
        class MySimulation(Simulation):
            def createObject(*args, **kwargs):
                assert False
    assert 'old-style simulation API' in str(e)

def test_simulator_destruction():
    sim = DummySimulator()
    scene = sampleSceneFrom('ego = new Object')
    sim.simulate(scene, maxSteps=1)
    sim.destroy()
    with pytest.raises(RuntimeError) as e:
        sim.simulate(scene, maxSteps=1)
    assert 'simulator cannot run additional simulations' in str(e)
    with pytest.raises(RuntimeError) as e:
        sim.destroy()
    assert 'destroy() called twice' in str(e)
