# arena_and_tables.scenic
"""Example demonstrating arena and table support in RoboSuite."""
model scenic.simulators.robosuite.model

# Example 1: Using TableArena (built-in table)
arena = TableArena

# Place robot near the table
robot = new PandaRobot at (-0.5, 0, 0)

# Place objects on the table (table height is 0.8m)
cube1 = new Cube at (0.1, 0.1, 0.85),
    with color (0.8, 0.2, 0.2)

cube2 = new Cube at (-0.1, 0.1, 0.85),
    with color (0.2, 0.8, 0.2)

ball = new Ball at (0, -0.1, 0.85),
    with color (0.2, 0.2, 0.8)

ego = robot

# Behavior to pick up objects
behavior PickAndPlace(robot):
    positions = [0, -0.5, 0, -2.0, 0, 1.5, 0.785, 0.04, 0.04]
    take SetJointPositions(positions)