# test_robosuite_basic.py
"""Test basic RoboSuite functionality without Scenic."""

import numpy as np
import robosuite as suite

# Create environment instance
env = suite.make(
    env_name="Lift",
    robots="Panda",
    has_renderer=True,
    has_offscreen_renderer=False,
    use_camera_obs=False,
)

# Reset the environment
env.reset()

print("Environment created successfully")
print(f"Action space: {env.action_spec}")
print(f"Robot name: {env.robots[0].name}")

# Run simulation
for i in range(100):
    action = np.random.randn(*env.action_spec[0].shape) * 0.1
    obs, reward, done, info = env.step(action)
    env.render()

print("Test complete")