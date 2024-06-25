# examples/robosuite/robot_physics_test.scenic
"""Test robot holds position while objects fall due to gravity."""

model scenic.simulators.robosuite.model

# Place robot at origin
panda = new Robot at (0, 0, 0),
    with robot_type "Panda"

# Place cube in air next to robot - should fall
cube = new Cube at (0.5, 0, 0.5),
    with width 0.1,
    with length 0.1, 
    with height 0.1,
    with color (1, 0, 0)  # Red

monitor TestMonitor():
    for i in range(10):
        wait
        print(f"Step {i}: Cube at z={cube.position.z:.3f}, Robot base at z={panda.position.z:.3f}")