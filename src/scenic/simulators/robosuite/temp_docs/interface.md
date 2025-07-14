# RoboSuite-Scenic Integration Documentation

## Installation

```bash
# Install Scenic
pip install scenic

# Install RoboSuite and MuJoCo
pip install robosuite
pip install mujoco
```

## Quick Start

```scenic
# Basic object placement
model scenic.simulators.robosuite.model

cube = new Cube at (0, 0, 0.5),
    with color (1, 0, 0)

ball = new Ball at (0.3, 0, 0.5),
    with color (0, 1, 0)

terminate after 50 steps
```

## API Reference

### Objects

#### `RoboSuiteObject`
Base class with physics properties:
- `density`: float (default: 1000)
- `friction`: tuple (default: (1.0, 0.005, 0.0001))
- `solref`: tuple (default: (0.02, 1.0))
- `solimp`: tuple (default: (0.9, 0.95, 0.001, 0.5, 2.0))

#### Basic Shapes
- `Cube`: 5cm cube, red default
- `Ball`: 5cm sphere, green default  
- `Cylinder`: 5cm diameter, 10cm height, blue default

#### `XMLObject`
Custom objects via XML:
- `xml_path`: Path to XML file
- `xml_string`: Inline XML string
- `material`: Dict with texture/material properties

### Robots

#### `Robot`
Base robot class:
- `robot_type`: string (default: "Panda")
- `initial_qpos`: list of joint positions
- `joint_positions`: dynamic property (read-only)

#### Implementations
- `PandaRobot`: 7-DOF Franka Panda
- `SawyerRobot`: 7-DOF Rethink Sawyer

### Arenas

- `EmptyArena`: Floor only
- `TableArena`: Standard table (1x0.8m at 0.8m height)
- `CustomArena`: From XML string/path

### Actions

#### `SetJointPositions(positions)`
Set robot joint angles directly.

## Tutorial

### 1. Basic Scene Creation

```scenic
model scenic.simulators.robosuite.model

# Create robot
robot = new PandaRobot at (0, -0.8, 0)

# Add objects
red_cube = new Cube at (0.2, 0, 0.5),
    with color (1, 0, 0)

# Run simulation
terminate after 100 steps
```

### 2. Physics Demonstration

```scenic
# Tower collapse example
model scenic.simulators.robosuite.model

# Stack cylinders with slight offsets
for i in range(6):
    new Cylinder at (Range(-0.01, 0.01), Range(-0.01, 0.01), 0.1 + i*0.121),
        with color Uniform((1,0,0), (0,1,0), (0,0,1))

terminate when (number of cylinders with z < 0.05) >= 3
```

### 3. Custom Objects

```scenic
# Using XML objects
model scenic.simulators.robosuite.model

class CustomBox(XMLObject):
    xml_string: '''
    <mujoco model="custom_box">
        <body name="box_main">
            <geom name="box_collision" type="box" size="0.05 0.05 0.05"/>
        </body>
    </mujoco>
    '''

box = new CustomBox at (0, 0, 0.5)
```

### 4. Robot Control

```scenic
# Joint position control
model scenic.simulators.robosuite.model

robot = new PandaRobot at (0, -0.8, 0)

behavior MoveJoints():
    take SetJointPositions([0, -0.5, 0, -2.0, 0, 1.5, 0.785])
    wait for 20 steps

robot.behavior = MoveJoints()
```

## Properties and Monitoring

### Reading Properties
- `position`: 3D position vector
- `velocity`: 3D velocity vector
- `speed`: Scalar speed
- `yaw`, `pitch`, `roll`: Euler angles

### Recording Data
```scenic
record cube.position as "cube_trajectory"
record robot.joint_positions as "joint_angles"
```

### Monitoring
```scenic
monitor PhysicsMonitor():
    for i in range(50):
        print(f"Height: {cube.position.z}")
        wait
```

## Simulator Parameters

```scenic
# Configure simulator
simulator RobosuiteSimulator(
    render=True,      # Enable visualization
    real_time=True,   # Real-time physics
    speed=2.0         # Simulation speed multiplier
)

# Set timestep
param timestep = 0.1  # Scenic timestep (seconds)
```

## Examples

See the `examples/robosuite/` directory for:
- `basic object placement/`: Shape and formation examples
- `physics_test/`: Physics demonstrations
- `tower_collapse.scenic`: Complex physics scenario
- `xml_test.scenic`: Custom object examples

## Limitations

Current implementation supports:
- Basic object placement and physics
- Robot positioning and joint control
- Custom objects via XML (Work In Progress)

Not yet implemented:
- Gripper control
- Position-based (OSC) control
- Task environments (Lift, Stack, etc.)
- Manipulation behaviors