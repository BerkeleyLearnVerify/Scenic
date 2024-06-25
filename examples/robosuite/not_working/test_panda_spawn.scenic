"""Test spawning a Panda robot at a specific position."""

model scenic.simulators.robosuite.model

# Create a Panda robot at a specific position
panda = new Panda at (0, 0, 0.5)

# Also create a visible cube to see if objects work
cube = new Cube at (0.2, 0, 0.5),
    with color (1, 0, 0)

# Run for just a few steps to see if it works
terminate after 10 seconds