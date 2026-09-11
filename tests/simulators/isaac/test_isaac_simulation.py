"""Tests that run real simulations in Isaac Sim, skipped when it isn't installed.

Run these through the Python environment Isaac Sim itself provides (see the
"Running Scenic with Isaac Sim" section of the interface's README), e.g.::

    isaac-sim/python.sh -m pytest tests/simulators/isaac/test_isaac_simulation.py

Isaac Sim takes tens of seconds to boot and supports only one
``SimulationApp`` per OS process, with no supported way to close and reopen
one within a process. So `isaacBackend` below is a session-scoped fixture:
every test shares the one `SimulationApp` it boots, and none of them (nor the
fixture) ever closes it -- `IsaacBackend` already registers an ``atexit`` hook
that does so when the interpreter exits.

For the same reason, every test forces the session's single auto-detected
backend, overriding any ``isaacBackend`` an example file sets for itself:
each named backend (``core_51``, ``experimental_60``, ...) independently
launches its own ``SimulationApp`` on first use, so using two different
backends within one process crashes the whole interpreter (segfault) rather
than raising a catchable error.
"""

import os

import pytest

pytest.importorskip("isaacsim")

import scenic
from tests.utils import sampleScene

EXAMPLES = os.path.join(
    os.path.dirname(__file__), "..", "..", "..", "examples", "isaacsim"
)


@pytest.fixture(scope="session")
def isaacBackend():
    """The single Isaac Sim backend/``SimulationApp`` shared by every test here."""
    from scenic.simulators.isaac.backends import getBackend

    backend = getBackend()
    backend.getSimulationApp(headless=True)
    return backend


def _simulate(scenario, maxSteps, **kwargs):
    """Sample a scene and run it to completion."""
    scene = sampleScene(scenario, maxIterations=500)
    simulator = scenario.getSimulator()
    simulation = simulator.simulate(scene, maxSteps=maxSteps, verbosity=1, **kwargs)
    assert simulation is not None, "simulation was rejected"
    return simulation


def _loadExample(relpath, isaacBackend, **params):
    params.setdefault("headless", True)
    params.setdefault("isaacBackend", isaacBackend.name)
    return scenic.scenarioFromFile(os.path.join(EXAMPLES, relpath), params=params)


def test_backend_matches_installed_isaac_sim(isaacBackend):
    """The auto-detected backend must actually be usable with the running Isaac Sim.

    Regression test for the "auto" backend only correctly guessing
    experimental_60 by luck of its fallback default: Isaac Sim's own bundled
    Python has no pip metadata for the ``isaacsim`` package, which used to
    make detection silently fall back regardless of the real version.
    """
    from isaacsim.core.version import get_version

    major = get_version()[0]
    expected = {"5": {"core_51", "experimental_51"}, "6": {"experimental_60"}}
    assert isaacBackend.name in expected.get(major, {isaacBackend.name})


def test_wheeled_and_manipulator_robots(isaacBackend, loadLocalScenario):
    """Regression test for the model.scenic/backend attribute-scoping bugs where
    a wheeled or plain object had no ``manipulatorProfile`` and a plain object
    had no ``wheelController``: both crashed with an AttributeError on the very
    first object created, before completing even one simulation step.

    Exercises: a mesh-shaped generic object, the built-in Create3 wheeled
    robot, a custom robot with a hand-written ``control`` function, and a
    FrankaPanda manipulator running the generic end-effector behaviors
    (OpenGripper/MoveEndEffectorTo/CloseGripper/HoldPosition).
    """
    scenario = loadLocalScenario(
        "basic.scenic", params={"headless": True, "isaacBackend": isaacBackend.name}
    )
    simulation = _simulate(scenario, maxSteps=150)
    assert simulation.result.terminationReason is not None


def test_environment_and_compressed_asset_mesh(isaacBackend):
    """Loads and converts an environment USD, places a new object on an
    existing prim using a compressed converted mesh (``.glb.bz2``) as its
    shape.

    Regression test for: flattening a referenced/payloaded USD asset before
    conversion (the old code exported only the root layer, breaking assets
    like this one built from references); the compressed mesh conversion and
    loading pipeline (`usd_conversion.convertUsdToMesh`, `utils.writeMesh`,
    `utils.loadAssetMesh`); and the existing-prim orientation fix (building
    the `Orientation` from the USD transform's rotation matrix instead of
    reinterpreting trimesh's static-XYZ Euler angles as Scenic's intrinsic
    Z-X-Y ones).
    """
    scenario = _loadExample("simple_room_asset_shape.scenic", isaacBackend)
    simulation = _simulate(scenario, maxSteps=10)
    assert simulation.result.terminationReason is not None


def test_manipulator_pick_and_place(isaacBackend):
    """Runs the UR5e Robotiq-gripper pick-and-place example (forcing the
    session's shared backend rather than the file's own ``experimental_51``;
    see module docstring). Regression test for the Robotiq USD authoring
    extracted into ``backends/robotiq.py`` and for the quaternion/Euler-angle
    math rewritten to use scipy's ``Rotation`` and Scenic's ``Orientation``.
    """
    scenario = _loadExample("robot/ur5e_example_experimental.scenic", isaacBackend)
    simulation = _simulate(scenario, maxSteps=150)
    assert simulation.result.terminationReason is not None
