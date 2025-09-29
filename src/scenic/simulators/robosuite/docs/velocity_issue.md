# RoboSuite Dynamic Property Fetching - Implementation Status and Issues

## Overview
This document details the implementation of dynamic property fetching from RoboSuite/MuJoCo for Scenic objects, addressing the PR comment: "This doesn't properly update the other dynamic properties like yaw, pitch, roll, velocity, etc. -- those need to have their current values looked up in robosuite."

## Successfully Implemented Properties

### Position Tracking ✅
- **Method**: `_get_object_position()`
- **Implementation**: Fetches from `data.xpos[body_id]`
- **Status**: Working correctly for all object types

### Orientation Properties ✅
- **Methods**: `_get_object_orientation()` for yaw, pitch, roll
- **Implementation**: 
  - Fetches quaternion from `data.xquat[body_id]`
  - Converts quaternion [w,x,y,z] to Euler angles (yaw, pitch, roll)
  - Uses proper ZXY rotation order for Scenic compatibility
- **Status**: Working correctly

### Robot-Specific Properties ✅
- **Properties**: jointPositions, eefPos, gripperState
- **Implementation**: Fetches from RoboSuite's robot observations
- **Status**: Working correctly

## Unresolved Issue: Velocity Tracking for Free-Joint Objects

### Problem Description
Velocity and angular velocity properties return zero for manipulable objects with free joints, even when objects are clearly moving (e.g., falling boxes).

### Test Results
```
Manual calculation: -30.866 m/s (actual falling velocity)
vel_from_sim: 0.000 (incorrect)
```

### Attempted Solutions

#### Attempt 1: Using `data.cvel` (Original Implementation)
```python
def _get_object_velocity(self, obj) -> Vector:
    body_id = self.body_id_map[obj_name]
    vel = self.data.cvel[body_id][:3]
    return Vector(vel[0], vel[1], vel[2])
```
**Result**: Always returns zero. `data.cvel` is for composite rigid bodies in contact, not free-floating joints.

#### Attempt 2: Using `data.qvel` with Joint Lookup
```python
def _get_object_velocity(self, obj) -> Vector:
    # Find joint for this object
    for mobj in self.robosuite_env.mujoco_objects:
        if mobj.name == obj_name:
            joint_name = mobj.joint_name
            break
    
    # Get velocity from qvel
    joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    qvel_addr = self.model.jnt_dofadr[joint_id]
    vel = self.data.qvel[qvel_addr:qvel_addr+3]
    return Vector(vel[0], vel[1], vel[2])
```
**Result**: Still returns zero. Possible issues:
- Joint name lookup may not be finding the correct joint
- `mujoco_objects` list might not be accessible at the right time
- Indexing into `qvel` might be incorrect

### Root Cause Analysis
The issue appears to be in how RoboSuite sets up free joints for manipulable objects and how we're accessing their velocity data. Possible causes:

1. **Joint Name Mismatch**: The joint names we're looking for might not match what's actually in the model
2. **Timing Issue**: The `mujoco_objects` list might not be populated when we need it
3. **Index Calculation**: The `qvel` indices for free joints might need different calculation
4. **RoboSuite Integration**: There might be a mismatch between how RoboSuite manages these joints internally

### Workaround
For scenarios requiring velocity information, manual calculation from position differences works:
```python
manual_velocity = (current_pos - previous_pos) / timestep
```

## Initial Orientation Setting Issue

### Secondary Issue Discovered
Objects don't respect initial orientations set via `facing` specifier.

### Attempted Fix
```python
# In _reset_internal()
if hasattr(obj, 'orientation'):
    scenic_orientation = obj.orientation
    if hasattr(scenic_orientation, 'getQuaternion'):
        quat = list(scenic_orientation.getQuaternion())
    # ... fallback to Euler conversion
```
**Status**: Partially implemented but not fully tested. Not part of original PR scope.

## Recommendations

1. **For PR Approval**: The core request has been addressed for most properties. Position, orientation, and robot-specific properties now correctly fetch from RoboSuite.

2. **Future Work**:
   - Debug joint name resolution for free-joint objects
   - Investigate RoboSuite's internal joint management
   - Consider alternative velocity sources or calculation methods
   - Complete initial orientation setting implementation

3. **Known Limitations**: Document that velocity/angular velocity for manipulable objects currently requires manual calculation from position differences.

## Testing
Use `test_velocity_debug.scenic` to verify the issue and test any future fixes. The manual velocity calculation confirms objects are moving correctly in physics, just the velocity fetch is broken.
