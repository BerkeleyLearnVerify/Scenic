# examples/robosuite/custom_mjcf_table.scenic
model scenic.simulators.robosuite.model

# Define a complex table structure with multiple geometries
table_xml = """
<geom name="tabletop" type="box" size="0.4 0.3 0.02" pos="0 0 0" rgba="0.5 0.3 0.1 1"/>
<geom name="leg1" type="cylinder" size="0.02 0.3" pos="-0.35 -0.25 -0.3" rgba="0.4 0.2 0.05 1"/>
<geom name="leg2" type="cylinder" size="0.02 0.3" pos="0.35 -0.25 -0.3" rgba="0.4 0.2 0.05 1"/>
<geom name="leg3" type="cylinder" size="0.02 0.3" pos="-0.35 0.25 -0.3" rgba="0.4 0.2 0.05 1"/>
<geom name="leg4" type="cylinder" size="0.02 0.3" pos="0.35 0.25 -0.3" rgba="0.4 0.2 0.05 1"/>
"""

# Define a sphere/ball using MJCF XML
ball_xml = """
<geom name="sphere" type="sphere" size="0.03" rgba="0 0 1 1"/>
"""

# Custom MJCF table placed on the floor
custom_table = new MJCFObject at (1.6, 0, 0.8),
    with mjcf_xml table_xml,
custom_table2 = new MJCFObject at (0.6, 0, 0.8),
    with mjcf_xml table_xml, 

# Red cube on the table
red_cube = new Box at (0.6, 0, 0.9),
    with color (1, 0, 0, 1),
    with width 0.04, with length 0.04, with height 0.04

# Blue ball made with MJCF XML on the floor
blue_ball = new MJCFObject at (0.6, 0, 0.94),
    with mjcf_xml ball_xml,

# Robot - positioned away from the table
ego = new Panda at (0, 0, 0)