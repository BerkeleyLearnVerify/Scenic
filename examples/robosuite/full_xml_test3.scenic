# examples/robosuite/full_xml.scenic
"""Currently No support for custom_arena_xml. fix: WIP"""
"""Test custom arena and custom objects with complete MJCF XML."""
model scenic.simulators.robosuite.model

# Define the complete custom arena XML
custom_arena_xml = """
<mujoco model="custom_arena">
  <asset>
    <texture builtin="gradient" height="256" rgb1=".9 .9 1." rgb2=".2 .3 .4" type="skybox" width="256"/>
    <texture builtin="flat" height="512" name="texplane" rgb1="0.7 0.7 0.8" rgb2="0.5 0.5 0.6" type="2d"/>
    <material name="floorplane" reflectance="0.01" shininess="0.0" specular="0.0" texrepeat="2 2" texture="texplane" texuniform="true"/>
  </asset>
  <worldbody>
    <!-- Floor -->
    <geom condim="3" material="floorplane" name="floor" pos="0 0 0" size="3 3 0.125" type="plane" group="1"/>
    
    <!-- Light -->
    <light pos="1.0 1.0 1.5" dir="-0.2 -0.2 -1" specular="0.3 0.3 0.3" directional="true" castshadow="false"/>
    
    <!-- Camera -->
    <camera name="frontview" pos="2 2 2" xyaxes="-1 1 0 -0.3 -0.3 1"/>
    
    <!-- GREEN TABLE to confirm custom arena is being used -->
    <body name="custom_table" pos="0.5 0 0.82">
      <!-- Visual geoms (group 1) - GREEN -->
      <geom name="table_top" type="box" size="0.4 0.5 0.025" rgba="0.2 0.9 0.2 1" group="1"/>
      <geom name="table_leg1" type="cylinder" pos="0.35 0.45 -0.4" size="0.02 0.4" rgba="0.1 0.7 0.1 1" group="1"/>
      <geom name="table_leg2" type="cylinder" pos="-0.35 0.45 -0.4" size="0.02 0.4" rgba="0.1 0.7 0.1 1" group="1"/>
      <geom name="table_leg3" type="cylinder" pos="0.35 -0.45 -0.4" size="0.02 0.4" rgba="0.1 0.7 0.1 1" group="1"/>
      <geom name="table_leg4" type="cylinder" pos="-0.35 -0.45 -0.4" size="0.02 0.4" rgba="0.1 0.7 0.1 1" group="1"/>
      
      <!-- Collision geoms (group 0) -->
      <geom name="table_top_col" type="box" size="0.4 0.5 0.025" rgba="0 0 0 0" group="0"/>
      <geom name="table_leg1_col" type="cylinder" pos="0.35 0.45 -0.4" size="0.02 0.4" rgba="0 0 0 0" group="0"/>
      <geom name="table_leg2_col" type="cylinder" pos="-0.35 0.45 -0.4" size="0.02 0.4" rgba="0 0 0 0" group="0"/>
      <geom name="table_leg3_col" type="cylinder" pos="0.35 -0.45 -0.4" size="0.02 0.4" rgba="0 0 0 0" group="0"/>
      <geom name="table_leg4_col" type="cylinder" pos="-0.35 -0.45 -0.4" size="0.02 0.4" rgba="0 0 0 0" group="0"/>
    </body>
  </worldbody>
</mujoco>
"""

# Define the ball object XML
ball_object_xml = """
<mujoco model="ball">
  <worldbody>
    <body>
      <body name="object">
        <geom name="visual" 
              type="sphere" 
              size="0.05" 
              rgba="1 1 0 1" 
              contype="0" 
              conaffinity="0" 
              group="1"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"""

# Create the custom arena
arena = new CustomArena,
    with arena_xml custom_arena_xml

# Create a yellow ball using custom XML
yellow_ball = new CustomObject at (0.5, 0.0, 1.0),
    with mjcf_xml ball_object_xml

# Create a red box using built-in Box class for comparison
red_box = new Box at (0.7, 0.2, 1.0),
    with color (1, 0, 0, 1),
    with width 0.04, with length 0.04, with height 0.04

# Create the Panda robot
ego = new Panda at (-0.5, 0, 0)