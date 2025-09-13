# examples/robosuite/vase_objects.scenic
model scenic.simulators.robosuite.model

# Vase approximation using thin cylinders (appears hollow from top)
vase_xml = """
<!-- Outer wall -->
<geom name="outer_base" type="cylinder" size="0.03 0.01" pos="0 0 0" rgba="0.7 0.5 0.3 1"/>
<geom name="outer_body" type="cylinder" size="0.035 0.06" pos="0 0 0.06" rgba="0.7 0.5 0.3 1"/>
<geom name="outer_neck" type="cylinder" size="0.025 0.02" pos="0 0 0.13" rgba="0.7 0.5 0.3 1"/>
<geom name="outer_rim" type="cylinder" size="0.04 0.01" pos="0 0 0.15" rgba="0.7 0.5 0.3 1"/>

<!-- Inner cavity (smaller, offset upward to create hollow effect) -->
<geom name="inner" type="cylinder" size="0.028 0.055" pos="0 0 0.065" rgba="0.2 0.2 0.2 0.3"/>
"""

# Bowl using ellipsoid cut in half (visually)
bowl_xml = """
<!-- Bottom hemisphere -->
<geom name="bowl_outer" type="ellipsoid" size="0.06 0.06 0.03" pos="0 0 0" rgba="0.3 0.5 0.8 1"/>
<!-- Inner darker area to suggest depth -->
<geom name="bowl_inner" type="ellipsoid" size="0.055 0.055 0.025" pos="0 0 0.005" rgba="0.2 0.3 0.6 0.5"/>
"""

# Pot with lid approximation
pot_xml = """
<!-- Pot body -->
<geom name="pot_body" type="cylinder" size="0.04 0.04" pos="0 0 0" rgba="0.4 0.4 0.4 1"/>
<!-- Pot rim -->
<geom name="pot_rim" type="cylinder" size="0.042 0.003" pos="0 0 0.04" rgba="0.35 0.35 0.35 1"/>
<!-- Handles -->
<geom name="handle1" type="capsule" size="0.003 0.015" pos="0.045 0 0.03" euler="0 1.57 0" rgba="0.3 0.3 0.3 1"/>
<geom name="handle2" type="capsule" size="0.003 0.015" pos="-0.045 0 0.03" euler="0 1.57 0" rgba="0.3 0.3 0.3 1"/>
<!-- Lid -->
<geom name="lid" type="cylinder" size="0.04 0.004" pos="0 0 0.045" rgba="0.45 0.45 0.45 1"/>
<geom name="lid_handle" type="sphere" size="0.008" pos="0 0 0.052" rgba="0.3 0.3 0.3 1"/>
"""

# Standard table
work_table = new Table at (0.6, 0, 0.8)

# Place objects
vase = new MJCFObject at (0.5, 0, 0.83),
    with mjcf_xml vase_xml

bowl = new MJCFObject at (0.65, 0, 0.83),
    with mjcf_xml bowl_xml

pot = new MJCFObject at (0.75, 0, 0.83),
    with mjcf_xml pot_xml

# Robot
ego = new Panda at (0, 0, 0)