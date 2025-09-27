"""Test with two XML tables and XML cubes on top, positioned in front of robot."""
model scenic.simulators.robosuite.model

# Get the directory of this scenic file for relative paths
param scenic_file_dir = localPath(".")

# Table 1 XML - a manipulable table object
table_xml = """
<mujoco model="table">
  <worldbody>
    <body>
      <body name="object">
        <!-- Table top -->
        <geom name="table_top_visual" 
              type="box" 
              size="0.4 0.3 0.025" 
              pos="0 0 0"
              rgba="0.6 0.3 0.1 1" 
              group="1"/>
        
        <!-- Table legs -->
        <geom name="leg1_visual" type="cylinder" pos="0.35 0.25 -0.4" size="0.02 0.4" rgba="0.5 0.25 0.05 1" group="1"/>
        <geom name="leg2_visual" type="cylinder" pos="-0.35 0.25 -0.4" size="0.02 0.4" rgba="0.5 0.25 0.05 1" group="1"/>
        <geom name="leg3_visual" type="cylinder" pos="0.35 -0.25 -0.4" size="0.02 0.4" rgba="0.5 0.25 0.05 1" group="1"/>
        <geom name="leg4_visual" type="cylinder" pos="-0.35 -0.25 -0.4" size="0.02 0.4" rgba="0.5 0.25 0.05 1" group="1"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"""

# Red cube XML
red_cube_xml = """
<mujoco model="red_cube">
  <worldbody>
    <body>
      <body name="object">
        <geom name="cube_visual" 
              type="box" 
              size="0.03 0.03 0.03" 
              pos="0 0 0"
              rgba="0.9 0.1 0.1 1" 
              group="1"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"""

# Green cube XML
green_cube_xml = """
<mujoco model="green_cube">
  <worldbody>
    <body>
      <body name="object">
        <geom name="cube_visual" 
              type="box" 
              size="0.03 0.03 0.03" 
              pos="0 0 0"
              rgba="0.1 0.9 0.1 1" 
              group="1"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"""


# Create the first table (brown) - closer to robot
table1 = new CustomObject on arena_floor,
    with mjcf_xml table_xml

# Create the second table (blue) - farther from robot
table2 = new CustomObject on arena_floor,
    with mjcf_xml table_xml


red_cube = new CustomObject on table1,
    with mjcf_xml red_cube_xml

# Place green cube on second table
green_cube = new CustomObject on table2,
    with mjcf_xml green_cube_xml

# Robot at origin
ego = new Panda at (0, 0, 0)