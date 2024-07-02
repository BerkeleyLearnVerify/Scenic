# xml_objects_example.scenic
"""Example demonstrating XML-based object support."""
model scenic.simulators.robosuite.model

# Example 1: Custom object from XML string
custom_box = new XMLObject at (0.2, 0, 0.85),
    with xml_string '''<?xml version="1.0" encoding="utf-8"?>
    <mujoco model="custom_box">
        <worldbody>
            <body name="custom_box_main">
                <geom name="box_geom" type="box" size="0.1 0.05 0.03" 
                      rgba="0.8 0.4 0.1 1" friction="1 0.005 0.0001"/>
                <geom name="box_visual" type="box" size="0.1 0.05 0.03" 
                      rgba="0.9 0.5 0.2 1" conaffinity="0" contype="0" group="1"/>
            </body>
        </worldbody>
    </mujoco>
    ''',
    with color (0.2, 0.8, 0.2)  # Override color to green

# Example 2: Complex shape from XML
pyramid = new CustomXMLObject at (-0.2, 0, 0.85),
    with xml_string '''<?xml version="1.0" encoding="utf-8"?>
    <mujoco model="pyramid">
        <worldbody>
            <body name="pyramid_main">
                <!-- Base -->
                <geom name="pyramid_base" type="box" pos="0 0 0" 
                      size="0.05 0.05 0.01" rgba="0.7 0.7 0.3 1"/>
                <!-- Middle -->
                <geom name="pyramid_middle" type="box" pos="0 0 0.02" 
                      size="0.03 0.03 0.01" rgba="0.7 0.7 0.3 1"/>
                <!-- Top -->
                <geom name="pyramid_top" type="box" pos="0 0 0.04" 
                      size="0.01 0.01 0.01" rgba="0.7 0.7 0.3 1"/>
            </body>
        </worldbody>
    </mujoco>
    '''

# Example 3: Decorative object
trophy = new XMLObject at (0, 0.2, 0.85),
    with xml_string '''<?xml version="1.0" encoding="utf-8"?>
    <mujoco model="trophy">
        <worldbody>
            <body name="trophy_main">
                <!-- Base -->
                <geom name="trophy_base" type="cylinder" pos="0 0 0" 
                      size="0.04 0.01" rgba="0.3 0.3 0.3 1"/>
                <!-- Stem -->
                <geom name="trophy_stem" type="cylinder" pos="0 0 0.03" 
                      size="0.01 0.02" rgba="0.8 0.7 0.2 1"/>
                <!-- Cup -->
                <geom name="trophy_cup1" type="sphere" pos="0 0 0.06" 
                      size="0.03" rgba="0.8 0.7 0.2 1"/>
                <geom name="trophy_cup2" type="cylinder" pos="0 0 0.07" 
                      size="0.025 0.01" rgba="0.8 0.7 0.2 1"/>
                <!-- Handles -->
                <geom name="trophy_handle1" type="capsule" fromto="-0.03 0 0.06 -0.05 0 0.05" 
                      size="0.005" rgba="0.8 0.7 0.2 1"/>
                <geom name="trophy_handle2" type="capsule" fromto="0.03 0 0.06 0.05 0 0.05" 
                      size="0.005" rgba="0.8 0.7 0.2 1"/>
            </body>
        </worldbody>
    </mujoco>
    ''',
    with color (0.9, 0.8, 0.1)  # Gold color

# Standard arena and robot
arena = TableArena
robot = new PandaRobot at (0, -0.5, 0)

ego = robot

# Behavior to inspect objects
behavior InspectObjects(robot):
    # Look at custom box
    pos1 = [0.3, -0.5, 0, -2.0, 0, 1.5, 0, 0.04, 0.04]
    take SetJointPositions(pos1)
    wait 2
    
    # Look at pyramid
    pos2 = [-0.3, -0.5, 0, -2.0, 0, 1.5, 0, 0.04, 0.04]
    take SetJointPositions(pos2)
    wait 2
    
    # Look at trophy
    pos3 = [0, -0.5, 0, -2.0, 0, 1.5, 0, 0.04, 0.04]
    take SetJointPositions(pos3)