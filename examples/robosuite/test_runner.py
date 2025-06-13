#!/usr/bin/env python3
# examples/robosuite/test_runner.py
"""Production test runner for Scenic-RoboSuite integration.

This script runs the validated working scenarios to verify
the Scenic-RoboSuite integration is functioning correctly.
"""

import sys
import time
import os
from pathlib import Path

import scenic
from scenic.simulators.robosuite import RobosuiteSimulator


def run_test(test_name: str, 
            scenario_file: str, 
            duration: int = 8,
            use_table: bool = False) -> bool:
   """Run a single test scenario."""
   print(f"\n{'='*60}")
   print(f"TEST: {test_name}")
   print(f"File: {scenario_file}")
   print(f"Duration: {duration}s")
   print('='*60)
   
   try:
       # Check if file exists
       if not os.path.exists(scenario_file):
           print(f"FAILED: Scenario file not found: {scenario_file}")
           return False
       
       # Load scenario
       scenario = scenic.scenarioFromFile(scenario_file)
       scene, _ = scenario.generate()
       
       print(f"Scene loaded successfully")
       print(f"Objects in scene: {len(scene.objects)}")
       
       # Print object details
       for i, obj in enumerate(scene.objects):
           color_str = ""
           if hasattr(obj, 'color'):
               if hasattr(obj.color, '__iter__'):
                   color_str = f" - Color: ({obj.color[0]:.2f}, {obj.color[1]:.2f}, {obj.color[2]:.2f})"
                   
           print(f"  {i+1}. {type(obj).__name__} at "
                 f"({obj.position.x:.2f}, {obj.position.y:.2f}, {obj.position.z:.2f})"
                 f"{color_str}")
       
       # Create simulator using your working configuration
       simulator = RobosuiteSimulator(
           use_table=use_table,
           has_renderer=True
       )
       
       print(f"Simulator created")
       
       # Create simulation
       simulation = simulator.createSimulation(scene)
       print(f"Simulation initialized")
       
       # Run simulation
       print(f"\nRunning simulation for {duration} seconds...")
       steps_per_second = 50  # 50 FPS for smooth visualization
       total_steps = duration * steps_per_second
       
       for step in range(total_steps):
           simulation.step()
           
           # Progress indicator
           if step % (steps_per_second * 2) == 0:  # Every 2 seconds
               elapsed = step // steps_per_second
               print(f"  {elapsed}s / {duration}s elapsed...")
           
           time.sleep(0.02)  # 50 FPS
           
       # Clean up
       simulation.destroy()
       print(f"TEST PASSED: {test_name}")
       return True
       
   except Exception as e:
       print(f"TEST FAILED: {test_name}")
       print(f"   Error: {e}")
       import traceback
       traceback.print_exc()
       return False


def main():
   """Run the complete test suite."""
   print("SCENIC-ROBOSUITE INTEGRATION TEST SUITE")
   print("=" * 60)
   print("Testing the validated working scenarios.")
   print()
   
   # Your working test cases (from run_test_suite.py)
   test_cases = [
       {
           'name': 'Different Shapes (Cube, Ball, Cylinder)',
           'file': 'examples/robosuite/basic_shapes.scenic',
           'duration': 5,
           'use_table': False
       },
       {
           'name': '3x3 Grid Formation',
           'file': 'examples/robosuite/grid_formation.scenic', 
           'duration': 5,
           'use_table': False
       },
       {
           'name': 'Different Height Levels',
           'file': 'examples/robosuite/height_levels.scenic',
           'duration': 5,
           'use_table': False
       },
       {
           'name': 'Circular Formation',
           'file': 'examples/robosuite/circular_formation.scenic',
           'duration': 5,
           'use_table': False
       },
       {
           'name': 'Stacking Tower',
           'file': 'examples/robosuite/stacking_tower.scenic',
           'duration': 5,
           'use_table': False
       }
   ]
   
   # Run tests
   passed = 0
   total = len(test_cases)
   failed_tests = []
   
   for i, test_case in enumerate(test_cases, 1):
       print(f"\nRunning test {i}/{total}")
       
       success = run_test(
           test_case['name'],
           test_case['file'],
           test_case['duration'],
           test_case['use_table']
       )
       
       if success:
           passed += 1
       else:
           failed_tests.append(test_case['name'])
       
       # Pause between tests (except for last test)
       if i < total:
           print(f"\nTest {i} complete. Press Enter to continue to next test...")
           try:
               input()
           except KeyboardInterrupt:
               print("\n\nTest suite interrupted by user")
               break
   
   # Print final results
   print(f"\n\nTEST SUITE COMPLETE!")
   print("=" * 60)
   print(f"RESULTS: {passed}/{total} tests passed")
   
   if passed == total:
       print("ALL TESTS PASSED!")
       print("Scenic-RoboSuite integration is working correctly!")
       return 0
   else:
       print(f"{total - passed} test(s) failed:")
       for failed_test in failed_tests:
           print(f"   {failed_test}")
       print("\nCheck the error messages above for troubleshooting.")
       return 1


if __name__ == "__main__":
   try:
       exit_code = main()
       sys.exit(exit_code)
   except KeyboardInterrupt:
       print("\n\nTest suite interrupted by user")
       sys.exit(1)
   except Exception as e:
       print(f"\n\nUnexpected error in test suite: {e}")
       import traceback
       traceback.print_exc()
       sys.exit(1)