# examples/robosuite/egg_shape.scenic
model scenic.simulators.robosuite.model

# Proper egg shape using ellipsoid
egg_xml = """
<geom name="egg" type="ellipsoid" size="0.025 0.025 0.04" pos="0 0 0" rgba="0.95 0.9 0.85 1"/>
"""

# Standard table
work_table = new Table at (0.6, 0, 0.8)

# Single egg in center of table
egg = new MJCFObject at (0.6, 0, 0.85),
    with mjcf_xml egg_xml

# Reference cube for size comparison
cube = new Box at (0.5, 0, 0.83),
    with color (1, 0, 0, 1),
    with width 0.04, with length 0.04, with height 0.04

# Robot
ego = new Panda at (0, 0, 0)