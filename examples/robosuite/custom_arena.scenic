# custom_arena.scenic
"""Example using custom arena defined with XML."""
model scenic.simulators.robosuite.model

# Define custom arena with shelves
arena = new CustomArena with xml_string '''
<mujoco>
    <asset>
        <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="512"/>
        <texture name="texplane" type="2d" builtin="checker" rgb1=".25 .25 .25" rgb2=".3 .3 .3" width="512" height="512" mark="cross" markrgb=".8 .8 .8"/>
        <material name="matplane" reflectance="0.3" texture="texplane" texrepeat="1 1" texuniform="true"/>
    </asset>
    
    <worldbody>
        <!-- Floor -->
        <geom name="floor" pos="0 0 0" size="3 3 0.125" type="plane" material="matplane" condim="3"/>
        
        <!-- Shelf 1 -->
        <body name="shelf1" pos="-1.0 0.5 0">
            <!-- Base -->
            <geom name="shelf1_base" type="box" size="0.4 0.3 0.02" pos="0 0 0.5" rgba="0.7 0.5 0.3 1"/>
            <!-- Top shelf -->
            <geom name="shelf1_top" type="box" size="0.4 0.3 0.02" pos="0 0 1.0" rgba="0.7 0.5 0.3 1"/>
            <!-- Back -->
            <geom name="shelf1_back" type="box" size="0.02 0.3 0.5" pos="-0.4 0 0.75" rgba="0.7 0.5 0.3 1"/>
            <!-- Sides -->
            <geom name="shelf1_side1" type="box" size="0.4 0.02 0.5" pos="0 -0.3 0.75" rgba="0.7 0.5 0.3 1"/>
            <geom name="shelf1_side2" type="box" size="0.4 0.02 0.5" pos="0 0.3 0.75" rgba="0.7 0.5 0.3 1"/>
        </body>
        
        <!-- Shelf 2 -->
        <body name="shelf2" pos="1.0 0.5 0">
            <!-- Base -->
            <geom name="shelf2_base" type="box" size="0.4 0.3 0.02" pos="0 0 0.5" rgba="0.7 0.5 0.3 1"/>
            <!-- Top shelf -->
            <geom name="shelf2_top" type="box" size="0.4 0.3 0.02" pos="0 0 1.0" rgba="0.7 0.5 0.3 1"/>
            <!-- Back -->
            <geom name="shelf2_back" type="box" size="0.02 0.3 0.5" pos="-0.4 0 0.75" rgba="0.7 0.5 0.3 1"/>
            <!-- Sides -->
            <geom name="shelf2_side1" type="box" size="0.4 0.02 0.5" pos="0 -0.3 0.75" rgba="0.7 0.5 0.3 1"/>
            <geom name="shelf2_side2" type="box" size="0.4 0.02 0.5" pos="0 0.3 0.75" rgba="0.7 0.5 0.3 1"/>
        </body>
        
        <!-- Work table in center -->
        <body name="worktable" pos="0 0 0">
            <geom name="worktable_top" type="box" size="0.5 0.4 0.02" pos="0 0 0.8" rgba="0.8 0.8 0.8 1"/>
            <geom name="worktable_leg1" type="cylinder" size="0.02 0.4" pos="0.45 0.35 0.4" rgba="0.5 0.5 0.5 1"/>
            <geom name="worktable_leg2" type="cylinder" size="0.02 0.4" pos="-0.45 0.35 0.4" rgba="0.5 0.5 0.5 1"/>
            <geom name="worktable_leg3" type="cylinder" size="0.02 0.4" pos="0.45 -0.35 0.4" rgba="0.5 0.5 0.5 1"/>
            <geom name="worktable_leg4" type="cylinder" size="0.02 0.4" pos="-0.45 -0.35 0.4" rgba="0.5 0.5 0.5 1"/>
        </body>
    </worldbody>
</mujoco>
'''

# Place robot in front of work table
robot = new PandaRobot at (0, -0.8, 0)

# Objects on shelves
box1 = new Cube at (-1.0, 0.5, 0.55),
    with color (0.9, 0.1, 0.1)

box2 = new Cube at (1.0, 0.5, 0.55),
    with color (0.1, 0.9, 0.1)

# Object on work table
cylinder = new Cylinder at (0, 0, 0.85),
    with height 0.1,
    with color (0.1, 0.1, 0.9)

ego = robot

# Behavior to organize objects
behavior OrganizeWorkspace(robot):
    # Initial position
    initial = [0, -0.785, 0, -2.356, 0, 1.571, 0.785, 0.04, 0.04]
    take SetJointPositions(initial)
    wait 2
    
    # Reach for left shelf
    reach_left = [-1.2, -0.5, 0, -2.0, 0, 1.5, 0.785, 0.04, 0.04]
    take SetJointPositions(reach_left)
    wait 2
    
    # Reach for right shelf
    reach_right = [1.2, -0.5, 0, -2.0, 0, 1.5, -0.785, 0.04, 0.04]
    take SetJointPositions(reach_right)