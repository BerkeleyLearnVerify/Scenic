# xml_from_file.scenic
"""Example loading objects from XML files."""
model scenic.simulators.robosuite.model

# Note: These paths would need to point to actual XML files
# For demo purposes, we'll use xml_string instead

# Simulated "loaded from file" object
tool = new XMLObject at (0.1, 0.1, 0.85),
    with xml_string '''<?xml version="1.0" encoding="utf-8"?>
    <mujoco model="hammer">
        <worldbody>
            <body name="hammer_main">
                <!-- Handle -->
                <geom name="handle" type="cylinder" pos="0 0 0" size="0.01 0.05" 
                      rgba="0.6 0.4 0.2 1" friction="1.5 0.005 0.0001"/>
                <!-- Head -->
                <geom name="head" type="box" pos="0 0 0.06" size="0.015 0.04 0.015" 
                      rgba="0.3 0.3 0.3 1" density="8000"/>
                <!-- Visual details -->
                <geom name="handle_grip" type="cylinder" pos="0 0 -0.03" size="0.012 0.02" 
                      rgba="0.2 0.2 0.2 1" conaffinity="0" contype="0" group="1"/>
            </body>
        </worldbody>
    </mujoco>
    ''',
    with xml_size [0.08, 0.03, 0.12]  # Override size if needed

# Another complex object
gear = new CustomXMLObject at (-0.1, -0.1, 0.85),
    with xml_string '''<?xml version="1.0" encoding="utf-8"?>
    <mujoco model="gear">
        <worldbody>
            <body name="gear_main">
                <!-- Main disk -->
                <geom name="gear_disk" type="cylinder" pos="0 0 0" size="0.04 0.005" 
                      rgba="0.7 0.7 0.7 1"/>
                <!-- Center hole -->
                <geom name="gear_hole" type="cylinder" pos="0 0 0" size="0.01 0.006" 
                      rgba="0.3 0.3 0.3 1" conaffinity="0" contype="0" group="1"/>
                <!-- Teeth (simplified) -->
                <geom name="tooth1" type="box" pos="0.04 0 0" size="0.005 0.01 0.005" 
                      rgba="0.7 0.7 0.7 1"/>
                <geom name="tooth2" type="box" pos="-0.04 0 0" size="0.005 0.01 0.005" 
                      rgba="0.7 0.7 0.7 1"/>
                <geom name="tooth3" type="box" pos="0 0.04 0" size="0.01 0.005 0.005" 
                      rgba="0.7 0.7 0.7 1"/>
                <geom name="tooth4" type="box" pos="0 -0.04 0" size="0.01 0.005 0.005" 
                      rgba="0.7 0.7 0.7 1"/>
            </body>
        </worldbody>
    </mujoco>
    '''

# Mixed with standard objects
cube = new Cube at (0, 0, 0.85),
    with color (0.2, 0.2, 0.8)

# Setup
arena = TableArena
robot = new PandaRobot at (0, -0.5, 0)

ego = robot

# Behavior
behavior ManipulateTools(robot):
    # Initial position
    take SetJointPositions([0, -0.785, 0, -2.356, 0, 1.571, 0.785, 0.04, 0.04])
    wait 2
    
    # Reach for tool
    take SetJointPositions([0.2, -0.5, 0, -2.0, 0, 1.5, 0.785, 0.04, 0.04])
    wait 2
    
    # Reach for gear
    take SetJointPositions([-0.2, -0.5, 0, -2.0, 0, 1.5, -0.785, 0.04, 0.04])