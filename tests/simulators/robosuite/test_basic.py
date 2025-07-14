# tests/simulators/robosuite/test_basic.py
"""Basic tests for Scenic-RoboSuite integration.

These tests are designed to run in headless environments like GitHub Actions.
Simplified to avoid implementation-specific assumptions.
"""

import unittest
import scenic
from scenic.simulators.robosuite import RobosuiteSimulator


class TestRobosuiteIntegration(unittest.TestCase):
    """Basic integration tests for RoboSuite simulator."""
    
    def test_simulator_creation(self):
        """Test that simulator can be created without renderer."""
        simulator = RobosuiteSimulator(has_renderer=False)
        self.assertIsNotNone(simulator)
        # Don't test internal attributes - just test it was created
    
    def test_basic_scenario_loading(self):
        """Test loading a basic scenario."""
        # Create basic test scenario inline (no external file dependencies)
        basic_scenario = """
class TestCube(Object):
    width: 0.05
    length: 0.05
    height: 0.05

new TestCube at (0.0, 0.0, 0.1), with color (1, 0, 0)
"""
        scenario = scenic.scenarioFromString(basic_scenario)
        scene, _ = scenario.generate()
        
        self.assertEqual(len(scene.objects), 1)
        obj = scene.objects[0]
        self.assertEqual(type(obj).__name__, 'TestCube')
        self.assertAlmostEqual(obj.position.z, 0.1)
    
    def test_multiple_objects_scenario(self):
        """Test scenario with multiple objects of different types."""
        multi_scenario = """
class TestCube(Object):
    width: 0.05
    length: 0.05
    height: 0.05

class TestBall(Object):
    width: 0.04
    length: 0.04
    height: 0.04

class TestCylinder(Object):
    width: 0.06
    length: 0.06
    height: 0.08

cube = new TestCube at (-0.2, 0.0, 0.1), with color (1, 0, 0)
ball = new TestBall at (0.0, 0.0, 0.1), with color (0, 1, 0)  
cylinder = new TestCylinder at (0.2, 0.0, 0.1), with color (0, 0, 1)
"""
        scenario = scenic.scenarioFromString(multi_scenario)
        scene, _ = scenario.generate()
        
        self.assertEqual(len(scene.objects), 3)
        
        # Check object types
        object_types = [type(obj).__name__ for obj in scene.objects]
        self.assertIn('TestCube', object_types)
        self.assertIn('TestBall', object_types)
        self.assertIn('TestCylinder', object_types)
    
    def test_headless_simulation_creation(self):
        """Test creating simulation in headless mode."""
        test_scenario = """
class SimpleCube(Object):
    width: 0.04
    length: 0.04
    height: 0.04

new SimpleCube at (0.0, 0.0, 0.1)
"""
        scenario = scenic.scenarioFromString(test_scenario)
        scene, _ = scenario.generate()
        
        # Create headless simulator
        simulator = RobosuiteSimulator(
            has_renderer=False,
            has_offscreen_renderer=False,
            use_camera_obs=False
        )
        
        # Create simulation
        simulation = simulator.createSimulation(scene)
        self.assertIsNotNone(simulation)
        
        # Test that simulation can be stepped
        try:
            simulation.step()
            simulation.step()
            simulation.step()
        except Exception as e:
            self.fail(f"Simulation stepping failed: {e}")
        finally:
            # Clean up
            simulation.destroy()
    
    def test_shape_detection_if_utils_exist(self):
        """Test shape detection only if utils module exists."""
        try:
            from scenic.simulators.robosuite.utils import detect_object_shape
            
            # Test cube detection
            cube_scenario = """
class MyCube(Object):
    width: 0.05
    length: 0.05 
    height: 0.05

new MyCube at (0.0, 0.0, 0.1)
"""
            scenario = scenic.scenarioFromString(cube_scenario)
            scene, _ = scenario.generate()
            cube_obj = scene.objects[0]
            self.assertEqual(detect_object_shape(cube_obj), 'cube')
            
        except ImportError:
            # Skip this test if utils doesn't exist yet
            self.skipTest("Utils module not found - skipping shape detection test")
    
    def test_color_conversion_if_utils_exist(self):
        """Test color conversion only if utils module exists."""
        try:
            from scenic.simulators.robosuite.utils import scenic_to_rgba
            
            # Test RGB tuple
            rgba = scenic_to_rgba((1.0, 0.5, 0.0))
            self.assertEqual(len(rgba), 4)  # Should be RGBA
            self.assertEqual(rgba[:3], [1.0, 0.5, 0.0])  # RGB should match
            
            # Test that function returns some valid RGBA
            rgba2 = scenic_to_rgba((1.0, 0.5, 0.0, 0.8))
            self.assertEqual(len(rgba2), 4)  # Should be RGBA
            
        except ImportError:
            # Skip this test if utils doesn't exist yet
            self.skipTest("Utils module not found - skipping color conversion test")
    
    def test_object_dimensions_if_utils_exist(self):
        """Test object dimension extraction only if utils exists."""
        try:
            from scenic.simulators.robosuite.utils import get_object_dimensions
            
            dimension_scenario = """
class SizedObject(Object):
    width: 0.1
    length: 0.2
    height: 0.3

new SizedObject at (0.0, 0.0, 0.1)
"""
            scenario = scenic.scenarioFromString(dimension_scenario)
            scene, _ = scenario.generate()
            obj = scene.objects[0]
            
            width, length, height = get_object_dimensions(obj)
            self.assertAlmostEqual(width, 0.1)
            self.assertAlmostEqual(length, 0.2)
            self.assertAlmostEqual(height, 0.3)
            
        except ImportError:
            # Skip this test if utils doesn't exist yet
            self.skipTest("Utils module not found - skipping dimension test")


class TestRobosuiteBasicRequirements(unittest.TestCase):
    """Test basic requirement handling in RoboSuite scenarios."""
    
    def test_simple_fixed_positions(self):
        """Test scenario with simple fixed positions (no complex requirements)."""
        simple_scenario = """
class SimpleCube(Object):
    width: 0.05
    length: 0.05
    height: 0.05

cube1 = new SimpleCube at (-0.3, 0.0, 0.1)
cube2 = new SimpleCube at (0.3, 0.0, 0.1)
"""
        scenario = scenic.scenarioFromString(simple_scenario)
        scene, _ = scenario.generate()
        
        self.assertEqual(len(scene.objects), 2)
        
        cube1, cube2 = scene.objects
        # Just check that objects have reasonable positions
        self.assertAlmostEqual(cube1.position.z, 0.1)
        self.assertAlmostEqual(cube2.position.z, 0.1)


if __name__ == '__main__':
    # Run tests with minimal output for CI
    unittest.main(verbosity=1)