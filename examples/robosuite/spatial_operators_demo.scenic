# examples/robosuite/spatial_operators_demo.scenic
""" WIP """

"""
Demonstration of spatial operators with mesh-aware RoboSuite objects.
This example shows how Scenic can now use 'on' and other spatial operators
with RoboSuite objects thanks to mesh extraction.
"""
model scenic.simulators.robosuite.model

# Enable mesh extraction for spatial awareness
param extract_meshes = True
param scenic_file_dir = localPath(".")
param render = True
param camera_view = "frontview"

# Robot at origin
ego = new Panda at (0, 0, 0)

# Create a table - Scenic knows its surface geometry
main_table = new Table at (0.6, 0, 0.8),
    with width 1.0,
    with length 0.8

# Red box at table surface (Table position is already at surface height)
red_box = new Box at (0.6, -0.2, 0.85),
    with color (1, 0, 0, 1),
    with width 0.05, with length 0.05, with height 0.05

# Green ball also at table surface
green_ball = new Ball at (0.7, 0.1, 0.85),
    with color (0, 1, 0, 1),
    with radius 0.03

# Blue cylinder on top of the red box - stacking works now
blue_cylinder = new Cylinder on red_box,  # Stack on top of box
    with color (0, 0, 1, 1),
    with width 0.04, with length 0.04, with height 0.08

# Custom object with mesh
custom_xml = """
<mujoco model="custom_block">
  <worldbody>
    <body>
      <body name="object">
        <geom name="visual" 
              type="box" 
              size="0.08 0.04 0.02" 
              pos="0 0 0"
              rgba="0.8 0.8 0.2 1" 
              group="1"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"""

# Yellow block at table surface  
yellow_block = new CustomObject at (0.5, 0.2, 0.85),
    with mjcfXml custom_xml

# Complex object (Milk carton) at table surface
milk = new Milk at (0.8, -0.1, 0.85)

# Spatial constraint - require objects to be visible from robot
require ego can see red_box
require ego can see green_ball

# Behavior using spatial awareness
behavior PickupSequence():
    """Pick up objects in sequence based on their spatial positions."""
    # Check which object is closer
    if distance to red_box < distance to green_ball:
        do PickAndLift(red_box, height=1.1)
    else:
        do PickAndLift(green_ball, height=1.1)

ego.behavior = PickupSequence()