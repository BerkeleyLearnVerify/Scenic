"""Quick manual test to see if my basic RoboSuite connection works."""

# This test script should be run from the root of the Scenic repository.

import sys
import os

# This ensures that the local 'scenic' package is used for the import
sys.path.insert(0, os.path.abspath('./src'))

print("Testing basic RoboSuite connection...")

# Test 1: Can we import RoboSuite?
try:
    import robosuite as suite
    print("Successfully imported RoboSuite")
except Exception as e:
    print(f"Failed to import RoboSuite: {e}")
    exit(1)

# Test 2: Can we create a basic environment?
try:
    env = suite.make('Lift', robots=['Panda'], has_renderer=False, use_camera_obs=False)
    print("Successfully created RoboSuite Lift environment")
except Exception as e:
    print(f"Failed to create environment: {e}")
    exit(1)

# Test 3: Can we reset and step?
try:
    obs = env.reset()
    action = [0.0] * env.action_spec[0].shape[0]
    obs, reward, done, info = env.step(action)
    print("Successfully reset and stepped environment")
except Exception as e:
    print(f"Failed to step environment: {e}")

# Test 4: Can we import our simulator class?
try:
    # Corrected import statement:
    from scenic.simulators.robosuite.simulator import RobosuiteSimulator
    simulator = RobosuiteSimulator()
    print("Successfully imported and created RobosuiteSimulator")
except Exception as e:
    print(f"Failed with simulator: {e}")

# Cleanup
env.close()
print("Basic connection test completed!")