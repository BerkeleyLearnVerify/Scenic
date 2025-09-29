# RoboSuite-Scenic Interface Documentation

## Status: Prototype 

### Overview
This interface enables Scenic scenarios to run in the RoboSuite robotics simulator, allowing declarative specification of robotic manipulation tasks with physics simulation.

---

## Working Features

### Scene Creation
- **Robot Support**: Panda, UR5e, Jaco, IIWA robots
- **Basic Objects**: Box, Ball, Cylinder, Capsule primitives
- **Custom Objects**: MJCF XML with automatic collision generation
- **Tables**: Standard table support with MultiTableArena
- **Custom Arenas**: Full MJCF arena definitions

### Dynamic Simulation
- **Position Tracking**: Object positions update correctly from physics
- **Orientation Tracking**: Yaw/pitch/roll from MuJoCo quaternions
- **Robot Properties**: Joint positions, end-effector position, gripper state
- **Behaviors**: Pick-and-place behaviors with OSC control
- **Collision Detection**: Automatic collision geometry for all objects

### Asset Management
- **Mesh Loading**: STL, OBJ with automatic repair
- **Texture Support**: PNG, JPG (auto-converted)
- **Path Resolution**: 
  - Embedded XML: Paths relative to Scenic file location
  - External XML: Paths relative to XML file location (see [xml-file-path.md](src/scenic/simulators/robosuite/docs/xml-file-path.md))

---

## Partially Working

### Velocity Tracking
- **Status**: Returns zero for free-joint objects
- **Root Cause**: Incorrect access to MuJoCo's `data.qvel` array
- **Attempted Fixes**: Tried both `data.cvel` and `data.qvel` approaches
- **Known Bug**: Currently no working implementation for velocity tracking
- **Details**: See [velocity_issue.md](src/scenic/simulators/robosuite/docs/velocity_issue.md) for full investigation

### Initial Orientations
- **Status**: `facing` specifier ignored, objects start at zero rotation
- **Workaround**: Use behaviors to set orientation after creation

### Table Dimensions
- **Issue**: Height hardcoded at 0.85m, changing causes visual/physics mismatch
- **Root Cause**: MultiTableArena design deadlock between Scenic and RoboSuite
- **Details**: See [table-rendering-issue.md](src/scenic/simulators/robosuite/docs/table-rendering-issue.md)

---

## Known Issues

### Critical Bugs
1. **Free-joint velocity always zero**
   - MuJoCo `qvel` access not working for manipulable objects
   - No current workaround available
   - Velocity property returns incorrect zero values
   
2. **Multi-robot action handling**
   - Single `pending_robot_action` variable overwrites with multiple robots
   - Causes action dimension mismatch errors
   - **Solution designed**: See [multi-robot_handling.md](src/scenic/simulators/robosuite/docs/multi-robot_handling.md)

3. **Segmentation fault on exit**
   - Viewer cleanup issue
   - Occurs after simulation completes

### Memory & Performance
- **Memory leak**: Temporary directories not always cleaned
- **Import overhead**: Fixed - RoboSuite now only imports when needed (see [import_fix.md](src/scenic/simulators/robosuite/docs/import_fix.md))

### Limitations
- No sensor support (cameras, force sensors)
- Default arena components conflict with custom arenas
- Table dimensions cannot be dynamically modified

---

## Example Files Guide

### `examples/robosuite/custom_mjcf_table.scenic`
**Tests**: Custom MJCF XML object creation
- Creates tables as manipulable objects (incorrect approach)
- Demonstrates embedded XML with primitive geometries
- **Issues Found**: 
  - Tables created with free joints (fall over)
  - Missing collision geometry

### `examples/robosuite/dual_table_workspace.scenic`
**Tests**: Multiple table placement
- Two standard tables with objects
- Random object placement using Range
- **Issues Found**:
  - Table height positioning error (z=0.8 instead of 0.425)
  - Object placement height mismatch
  - Use of `at` causes collision issues

### `examples/robosuite/file_xml.scenic`
**Tests**: External XML file loading
- Loads MJCF from `custom_object/bread.xml`
- Path resolution from Scenic file location
- Standard Table usage (correct approach)

### `examples/robosuite/full_xml.scenic`
**Tests**: Custom arena with complete MJCF
- Full arena definition with floor, lights, camera
- Custom arena property (`arenaXml`)
- **Issues Found**:
  - Property name typo (`arena_xml` should be `arenaXml`)
  - Conflicts with default arena components
  - Missing collision geometry in ball object

### `examples/robosuite/mesh_test.scenic`
**Tests**: Mesh file loading (STL)
- Loads dragon mesh from `custom_object/dragon.stl`
- Tests relative path resolution in MJCF
- Mesh scaling and material application

### `examples/robosuite/spatial_operators_demo.scenic`
**Tests**: Spatial operators and object stacking
- Object stacking with `on` operator
- Visibility requirements (`can see`)
- Distance-based behavior decisions
- **Issues Found**:
  - Table positioning error
  - Undefined `Milk` class
  - Unused `extract_meshes` parameter

### `examples/robosuite/stack_lift.scenic`
**Tests**: Pick and lift behavior
- Object stacking (box on box)
- PickObject and LiftToHeight behaviors
- Conditional simulation termination
- **Issues Found**:
  - Table height override problem

---

## File Structure & Utilities

### Static Assets (`src/scenic/simulators/robosuite/utils/`)
Pre-extracted 3D meshes for Scenic visualization. These allow Scenic to render objects without needing RoboSuite running.

#### `arena_meshes/`
- `floor.glb`, `walls.glb` - Default arena components
- `arena_config.json` - Dimensions and metadata
- Generated by: `extract_arena.py`

#### `robot_meshes/`
- `Panda.glb`, `UR5e.glb`, `Jaco.glb`, `IIWA.glb` - Robot models
- `dimensions.txt` - Robot bounding box dimensions
- Generated by: `extract_robots.py`

#### `table_meshes/`
- `standard_table.glb` - Default table mesh (fixed dimensions)
- Note: Cannot be dynamically adjusted due to MultiTableArena constraints
- Generated by: `extract_table.py`

#### `table_components/`
- Individual table parts for future composite mesh support
- Currently unused - waiting for Scenic composite mesh feature
- Generated by: `extract_table_components.py`

### Extraction Scripts (`src/scenic/simulators/robosuite/utils/`)
These scripts extract geometry from RoboSuite's runtime buffers to create static meshes. Unlike the main interface (which parses MJCF XML), these access RoboSuite's compiled models directly.

#### Key Scripts
- **`mjcf_to_mesh.py`**: Standalone converter using trimesh primitives
- **`extract_*.py`**: Runtime extraction from RoboSuite buffers
- **`default.scenic`**: Test file to verify extracted mesh rendering

---

## Basic Usage

### Minimal Example
```scenic
model scenic.simulators.robosuite.model

table = new Table at (0.6, 0, 0.425)
box = new Box on table, with color (1, 0, 0, 1)
ego = new Panda on (0, 0, 0)

behavior PickBox():
    do PickAndLift(box, height=1.1)

ego = new Panda on (0, 0, 0),
    with behavior PickBox()
```

### Common Pitfalls to Avoid
1. **Positioning specifiers**: 
   - Use `on` for robots and ground-level objects to avoid collision issues
   - `at` positions by center, causing robots to appear inside ground in Scenic's visualization
   - Tables can use `at` since they're elevated, but ensure correct z-coordinate (0.425)
2. **Table position**: Always use z=0.425 for Table center
3. **Custom tables**: Use built-in Table class, not CustomObject
4. **Robot creation**: Use `on` for positioning to avoid ground collision issues

### Running Simulations
```bash
# Compile only (no RoboSuite import)
scenic my_scenario.scenic

# Run simulation
scenic my_scenario.scenic --simulate --count 1
```

### Regenerating Static Assets
```bash
cd src/scenic/simulators/robosuite/utils
python extract_robots.py
python extract_arena.py
python extract_table.py
```

---

## Required Fixes

### High Priority
- Fix velocity fetching from `data.qvel` for free joints
- Implement multi-robot action handling (design in src/scenic/simulators/robosuite/docs/)
- Resolve segmentation fault on cleanup

### Medium Priority  
- Apply initial orientations from `facing` specifier
- Remove hardcoded table dimensions (needs Scenic composite mesh support)
- Improve error messages and exception handling
- Fix example files (incorrect syntax usage)

### Future Work
- Sensor integration (RGB, depth, force)
- Support for joint constraints
- erformance optimization for mesh loading
- Comprehensive test suite

---

## Technical Details

### Why Static Mesh Extraction?
RoboSuite builds complex objects at runtime from multiple components. To enable Scenic's collision checking and visualization without running RoboSuite, we pre-extract these as single meshes. This avoids the circular dependency where Scenic needs geometry for collision checking before RoboSuite runs.

### Known Issues

1. **Velocity Data Access Pattern**
   - RoboSuite stores velocities in `qvel` array indexed by joint DOF
   - Our joint name lookup fails to find the correct mapping
   - Needs investigation of RoboSuite's joint naming conventions

2. **Table Rendering Deadlock**
   - MultiTableArena expects only thickness parameter
   - Scenic needs full geometry before simulation
   - Cannot synchronize dynamically without composite mesh support

3. **Multi-Robot Control**
   - Current single action buffer design
   - Needs per-robot action tracking
   - Must handle different action dimensions per robot type

---


## Documentation Files
- `src/scenic/simulators/robosuite/docs/velocity_issue.md` - Detailed velocity tracking investigation
- `src/scenic/simulators/robosuite/docs/table-rendering-issue.md` - Table dimension synchronization problem
- `src/scenic/simulators/robosuite/docs/multi-robot_handling.md` - Proposed multi-robot solution
- `src/scenic/simulators/robosuite/docs/xml-file-path.md` - Path resolution behavior
- `src/scenic/simulators/robosuite/docs/import_fix.md` - RoboSuite import optimization

---