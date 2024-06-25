# test_suite_5_stacking.scenic
"""Test Suite 5: Stacking Tower."""

class StackCube(Object):
    width: 0.1
    length: 0.1
    height: 0.1

# Build a tower that will fall and stack
base_cube = new StackCube at (0.0, 0.0, 0.05), with color (1, 0, 0)      # Red base
level_2 = new StackCube at (0.0, 0.0, 0.2), with color (1, 0.5, 0)      # Orange
level_3 = new StackCube at (0.0, 0.0, 0.35), with color (1, 1, 0)       # Yellow  
level_4 = new StackCube at (0.0, 0.0, 0.5), with color (0, 1, 0)        # Green
level_5 = new StackCube at (0.0, 0.0, 0.65), with color (0, 0, 1)       # Blue
top_cube = new StackCube at (0.0, 0.0, 0.8), with color (0.5, 0, 1)     # Purple top

ego = level_3
