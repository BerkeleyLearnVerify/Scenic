# examples/robosuite/mesh_dragon.scenic
model scenic.simulators.robosuite.model

# Define MJCF XML with mesh asset
# Note: MuJoCo textures work differently than STL textures - it won't use the texture folder automatically
dragon_xml = """
<mujoco>
    <asset>
        <mesh name="dragon_mesh" file="src/scenic/simulators/robosuite/39-stl/stl/Dragon 2.5_stl.stl" scale="0.001 0.001 0.001"/>
    </asset>
    <body>
        <geom name="dragon" type="mesh" mesh="dragon_mesh" rgba="0.8 0.6 0.4 1"/>
    </body>
</mujoco>
"""

# Simpler test - just the body with mesh reference (in case full mujoco tag doesn't work)
dragon_body_xml = """
<body>
    <geom name="dragon" type="mesh" mesh="dragon_mesh" rgba="0.8 0.6 0.4 1"/>
</body>
"""

# Standard table
work_table = new Table at (0.6, 0, 0.8)

# Try loading the dragon mesh
dragon = new MJCFObject at (0.6, 0, 0.85),
    with mjcf_xml dragon_body_xml,
    with mjcf_name "dragon"

# Reference cube
cube = new Box at (0.5, 0, 0.83),
    with color (1, 0, 0, 1),
    with width 0.04, with length 0.04, with height 0.04

# Robot
ego = new Panda at (0, 0, 0)