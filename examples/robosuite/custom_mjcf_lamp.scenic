# examples/robosuite/custom_mjcf_lamp.scenic
model scenic.simulators.robosuite.model

# Define a complex table structure with multiple geometries
table_xml = """
<geom name="tabletop" type="box" size="0.4 0.3 0.02" pos="0 0 0" rgba="0.5 0.3 0.1 1"/>
<geom name="leg1" type="cylinder" size="0.02 0.3" pos="-0.35 -0.25 -0.3" rgba="0.4 0.2 0.05 1"/>
<geom name="leg2" type="cylinder" size="0.02 0.3" pos="0.35 -0.25 -0.3" rgba="0.4 0.2 0.05 1"/>
<geom name="leg3" type="cylinder" size="0.02 0.3" pos="-0.35 0.25 -0.3" rgba="0.4 0.2 0.05 1"/>
<geom name="leg4" type="cylinder" size="0.02 0.3" pos="0.35 0.25 -0.3" rgba="0.4 0.2 0.05 1"/>
"""

# Improved desk lamp with better proportions
lamp_xml = """
<!-- Heavy circular base -->
<geom name="base" type="cylinder" size="0.06 0.01" pos="0 0 0" rgba="0.1 0.1 0.1 1"/>
<!-- Base weight (inner cylinder for stability) -->
<geom name="base_weight" type="cylinder" size="0.04 0.008" pos="0 0 0.01" rgba="0.15 0.15 0.15 1"/>
<!-- Main vertical pole -->
<geom name="pole" type="cylinder" size="0.006 0.12" pos="0 0 0.12" rgba="0.7 0.7 0.7 1"/>
<!-- Adjustable joint at top of pole -->
<geom name="upper_joint" type="sphere" size="0.01" pos="0 0 0.24" rgba="0.3 0.3 0.3 1"/>
<!-- Angled arm extending from pole -->
<geom name="arm" type="cylinder" size="0.005 0.08" pos="0.04 0 0.27" euler="0 -0.5 0" rgba="0.7 0.7 0.7 1"/>
<!-- Lamp shade mount -->
<geom name="shade_mount" type="sphere" size="0.008" pos="0.08 0 0.3" rgba="0.3 0.3 0.3 1"/>
<!-- Conical lamp shade (using tapered cylinder) -->
<geom name="shade_top" type="cylinder" size="0.04 0.001" pos="0.08 0 0.32" rgba="0.9 0.85 0.7 1"/>
<geom name="shade_middle" type="cylinder" size="0.06 0.04" pos="0.08 0 0.28" rgba="0.9 0.85 0.7 0.9"/>
<geom name="shade_bottom" type="cylinder" size="0.08 0.001" pos="0.08 0 0.24" rgba="0.9 0.85 0.7 1"/>
"""

# Improved toolbox with better detail
toolbox_xml = """
<!-- Main box body -->
<geom name="body" type="box" size="0.12 0.07 0.05" pos="0 0 0" rgba="0.8 0.2 0.2 1"/>
<!-- Handle (curved) -->
<geom name="handle_left" type="cylinder" size="0.004 0.01" pos="-0.04 0 0.055" rgba="0.1 0.1 0.1 1"/>
<geom name="handle_middle" type="cylinder" size="0.004 0.04" pos="0 0 0.065" euler="0 1.57 0" rgba="0.1 0.1 0.1 1"/>
<geom name="handle_right" type="cylinder" size="0.004 0.01" pos="0.04 0 0.055" rgba="0.1 0.1 0.1 1"/>
<!-- Hinges -->
<geom name="hinge1" type="cylinder" size="0.003 0.015" pos="-0.08 0 0.04" euler="1.57 0 0" rgba="0.4 0.4 0.4 1"/>
<geom name="hinge2" type="cylinder" size="0.003 0.015" pos="0.08 0 0.04" euler="1.57 0 0" rgba="0.4 0.4 0.4 1"/>
<!-- Front latch -->
<geom name="latch" type="box" size="0.015 0.008 0.012" pos="0 -0.07 0.035" rgba="0.4 0.4 0.4 1"/>
"""

# Custom MJCF table
custom_table = new MJCFObject at (0.6, 0, 0.8),
    with mjcf_xml table_xml

# Improved desk lamp on the table
desk_lamp = new MJCFObject at (0.5, 0.15, 0.83),
    with mjcf_xml lamp_xml

# Toolbox on the table
toolbox = new MJCFObject at (0.7, -0.1, 0.83),
    with mjcf_xml toolbox_xml

# Simple red cube for scale reference
red_cube = new Box at (0.6, 0, 0.83),
    with color (1, 0, 0, 1),
    with width 0.04, with length 0.04, with height 0.04

# Robot
ego = new Panda at (0, 0, 0)