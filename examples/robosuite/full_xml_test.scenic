# examples/robosuite/full_xml_test.scenic
model scenic.simulators.robosuite.model

# Complex object with joints, multiple bodies, sites, etc.
complex_xml = """
<body>
    <inertial pos="0 0 0" mass="2.5" diaginertia="0.1 0.1 0.1"/>
    <joint name="main_hinge" type="hinge" axis="0 0 1" range="-45 45" damping="0.1"/>
    <geom name="base" type="cylinder" size="0.05 0.02" rgba="0.8 0.2 0.2 1"/>
    <site name="attachment_point" pos="0 0 0.02" size="0.01"/>
    
    <body name="arm" pos="0 0 0.02">
        <joint name="arm_joint" type="hinge" axis="1 0 0" range="-90 90"/>
        <geom name="arm_geom" type="capsule" size="0.01 0.1" pos="0 0 0.1" euler="0 0 0"/>
        <geom name="arm_visual" type="sphere" size="0.015" pos="0 0 0.2" rgba="0 1 0 0.5" group="1"/>
        
        <body name="end_effector" pos="0 0 0.2">
            <geom name="tip" type="sphere" size="0.02" rgba="0 0 1 1"/>
            <site name="tip_site" pos="0 0 0"/>
        </body>
    </body>
</body>
"""

# Standard table
work_table = new Table at (0.6, 0, 0.8)

# Complex object with full XML support
complex_object = new MJCFObject at (0.6, 0, 0.85),
    with mjcf_xml complex_xml,
    with mjcf_name "articulated_arm"

# You can also paste raw geom lists
multi_geom_xml = """
<geom type="box" size="0.05 0.05 0.01" pos="0 0 0" rgba="1 0 0 1"/>
<geom type="cylinder" size="0.02 0.05" pos="0 0 0.06" rgba="0 1 0 1"/>
<geom type="sphere" size="0.03" pos="0 0 0.12" rgba="0 0 1 1"/>
<site name="top" pos="0 0 0.15" size="0.01"/>
<joint type="ball" damping="0.01"/>
"""

stacked_object = new MJCFObject at (0.7, 0, 0.85),
    with mjcf_xml multi_geom_xml,
    with mjcf_name "stack"

# Robot
ego = new Panda at (0, 0, 0)