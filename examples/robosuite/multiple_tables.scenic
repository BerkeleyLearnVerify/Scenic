# multiple_tables.scenic
"""Example with multiple positionable tables."""
model scenic.simulators.robosuite.model

# Create multiple tables at different positions
table1 = new PositionableTable at (-0.8, 0, 0),
    with width 0.8,
    with length 0.6,
    with height 0.7

table2 = new PositionableTable at (0.8, 0, 0),
    with width 0.6,
    with length 0.6,
    with height 0.8

# Place robot between tables
robot = new PandaRobot at (0, -0.5, 0)

# Objects on table1
cube1 = new Cube at (-0.8, 0, 0.75),
    with color (1.0, 0.5, 0.0)  # Orange

# Objects on table2 
cylinder = new Cylinder at (0.8, 0, 0.85),
    with height 0.15,
    with color (0.5, 0.0, 1.0)  # Purple

ego = robot

# Behavior to move between tables
behavior TransferObjects(robot):
    # Move to first table
    positions1 = [-0.785, -0.5, 0, -2.0, 0, 1.5, 0.785, 0.04, 0.04]
    take SetJointPositions(positions1)
    wait 2
    
    # Move to second table
    positions2 = [0.785, -0.5, 0, -2.0, 0, 1.5, -0.785, 0.04, 0.04]
    take SetJointPositions(positions2)