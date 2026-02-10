import os
import numpy as np
import pytest
import scenic
from pathlib import Path

try:
    import mujoco
    from scenic.simulators.mujoco.simulator import MujocoSimulator
except ModuleNotFoundError:
    pytest.skip("MuJoCo package not installed.", allow_module_level=True)

from tests.utils import compileScenic, pickle_test, sampleScene, tryPickling

WINDOW_ERR = "Could not open window."

# Helper to run a simulation but skip cleanly on CI.
def simulate_or_skip(simulator, scene):
    try:
        return simulator.simulate(scene)
    except Exception as e:
        if WINDOW_ERR in str(e) or "display" in str(e).lower():
            pytest.skip("MuJoCo viewer cannot open on this platform/CI")
        else:
            raise

@pytest.fixture(scope="package")
def getMujocoSimulator():
    """Factory for creating MuJoCo simulators with/without viewer."""
    def _getMujocoSimulator(use_viewer=False):
        simulator = MujocoSimulator(use_viewer=use_viewer)
        return simulator
    
    yield _getMujocoSimulator

def test_basic_ground(getMujocoSimulator):
    """Test that a basic scene with ground can be created."""
    simulator = getMujocoSimulator(use_viewer=False)
    code = """
        model scenic.simulators.mujoco.model
        
        workspace = Workspace(RectangularRegion(0 @ 0, 0, 10, 10))
        
        ground = new Ground with width 10, with length 10
        
        terminate after 1 steps
    """
    scenario = compileScenic(code)
    scene = sampleScene(scenario)
    simulation = simulator.simulate(scene)
    assert simulation is not None
    assert len(simulation.result.trajectory) > 0

def test_ground_with_terrain(getMujocoSimulator):
    """Test ground with hills (terrain)."""
    simulator = getMujocoSimulator(use_viewer=False)
    code = """
        model scenic.simulators.mujoco.model
        
        workspace = Workspace(RectangularRegion(0 @ 0, 0, 20, 20))
        
        hill1 = new Hill at 2 @ 2, with width 3, with length 3, with height 0.5
        hill2 = new Hill at -2 @ -2, with width 2, with length 2, with height 0.3
        
        ground = new Ground with width 20, with length 20,
            with terrain (hill1, hill2),
            with gridSize 30
        
        terminate after 2 steps
    """
    scenario = compileScenic(code)
    scene = sampleScene(scenario)
    simulation = simulator.simulate(scene)
    assert simulation is not None
    # Verify ground was created.
    assert len(scene.objects) == 3  # 2 hills + 1 ground.

@pickle_test
def test_pickle(getMujocoSimulator):
    """Test that scenarios and scenes can be pickled."""
    code = """
        model scenic.simulators.mujoco.model
        
        workspace = Workspace(RectangularRegion(0 @ 0, 0, 10, 10))
        ground = new Ground with width 10, with length 10
        
        terminate after 1 steps
    """
    scenario = tryPickling(compileScenic(code))
    tryPickling(sampleScene(scenario))

def test_headless_execution(getMujocoSimulator):
    """Test that simulator can run completely headless without viewer."""
    simulator = getMujocoSimulator(use_viewer=False)
    code = """
        model scenic.simulators.mujoco.model
        
        workspace = Workspace(RectangularRegion(0 @ 0, 0, 10, 10))
        
        hill = new Hill at 0 @ 0, with width 5, with length 5, with height 1.0
        ground = new Ground with width 10, with length 10,
            with terrain (hill,),
            with gridSize 25
        
        terminate after 5 steps
    """
    scenario = compileScenic(code)
    scene = sampleScene(scenario)
    
    # This should NOT try to open a viewer window.
    simulation = simulator.simulate(scene)
    
    assert simulation is not None
    assert len(simulation.result.trajectory) == 6  # 5 steps + initial state.
    
def test_amiga_robot_integration(getMujocoSimulator):
    """Integration test: Loads the full Amiga example to verify robot XML assets."""
    
    current_dir = Path(__file__).parent
    repo_root = current_dir.parents[2]
    
    example_path = repo_root / "examples" / "mujoco" / "farm-ng" / "example_simulation.scenic"
    
    if not example_path.exists():
        pytest.skip(f"Amiga example not found at {example_path}")

    # Load the scenario using the 'scenic' library directly.
    # Override use_viewer=False to ensure it runs on CI/Headless.
    scenario = scenic.scenarioFromFile(
        str(example_path), 
        params={"use_viewer": False} 
    )
    
    # Generate a scene (this verifies 'amiga.scenic' parses correctly).
    scene, _ = scenario.generate()
    
    # Simulate for a few steps (this verifies 'amiga_base.xml' and physics load correctly).
    simulator = getMujocoSimulator(use_viewer=False)
    simulation = simulator.simulate(scene, maxSteps=5)
    
    assert simulation is not None
    assert len(simulation.result.trajectory) >= 6 # Initial state + 5 steps.