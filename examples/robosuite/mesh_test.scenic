# examples/robosuite/mesh_test.scenic
"""Test custom mesh objects with relative paths in XML."""
model scenic.simulators.robosuite.model

param scenic_file_dir = localPath(".")

# Dragon object XML - uses relative path to mesh file
dragon_object_xml = """
<mujoco model="dragon">
  <asset>
    <mesh name="dragon_mesh" file="custom_object/dragon.stl" scale="0.003 0.003 0.003"/>
    <material name="dragon_mat" rgba="0.8 0.2 0.2 1" specular="0.5" shininess="0.3"/>
  </asset>
  <worldbody>
    <body>
      <body name="object">
        <joint name="object_joint" type="free" damping="0.0005"/>
        
        <geom name="visual" 
              pos="0 0 0" 
              mesh="dragon_mesh" 
              type="mesh" 
              material="dragon_mat" 
              contype="0" 
              conaffinity="0" 
              group="1"/>
        
        <geom name="collision" 
              pos="0 0 0" 
              type="box" 
              size="0.04 0.04 0.01" 
              rgba="0 0 0 0" 
              group="0"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"""

# Use default table arena
table = new Table at (0.5, 0, 0.8)

# Create dragon - path in XML is relative to this .scenic file
dragon = new CustomObject at (0.5, 0, 0.88),
    with mjcf_xml dragon_object_xml

# Add a simple ball for comparison
ball = new Ball at (0.8, 0., 0.88),
    with color (0, 1, 0, 1),
    with radius 0.03

# Robot
ego = new Panda at (0, 0, 0)