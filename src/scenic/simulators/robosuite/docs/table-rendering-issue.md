# Table Rendering Issue Documentation

## Issue Summary
Tables appear correctly in robosuite but have dimension mismatches in Scenic's internal viewer when height is adjusted. Setting `height: 0.85` (full table height) causes robosuite to render a white block, while `height: 0.05` (tabletop thickness) causes Scenic to show a squished table.

## Technical Root Cause

### MultiTableArena Design
Robosuite's `MultiTableArena` provides full control over table elements as parameters without making tables free-joint objects. This design:
- Prevents tables from being physics-enabled manipulable objects
- Allows efficient arena-level table management
- Takes only tabletop thickness as the "size" parameter, not full height

### The Deadlock Problem
Synchronizing table rendering between simulators would require:
1. Running robosuite first to generate table geometry with Scenic configuration
2. Exporting that geometry back to Scenic for matching visualization
3. But Scenic needs the geometry *before* robosuite runs for collision checking

This circular dependency creates a deadlock that prevents dynamic synchronization.

## Current Solution
Static GLB files pre-built for standard table configurations:
- `utils/table_meshes/standard_table.glb` - Fixed geometry for Scenic
- Hardcoded thickness (0.05m) passed to robosuite's MultiTableArena
- Height property fixed at 0.85m for Scenic collision detection

## Limitations

### Dimension Mismatches
- **Height**: Primary breaking feature - changing causes visual inconsistencies
- **Width/Length**: Extreme values cause leg stretching in Scenic (not replicated in robosuite)
- Static nature means robosuite renders correctly but changes don't reflect in Scenic

### Configuration Restrictions
Users cannot customize table dimensions without breaking visual consistency between simulators.

## Potential Future Solution
Composite mesh assembly in Scenic:
- Separate meshes for tabletop and legs
- Unified into single object while retaining component individuality
- Would allow dynamic dimension adjustments

**Current blocker**: Scenic lacks this composite mesh feature where multiple meshes can be unified into one object with components maintaining their individual properties.

## Implementation Notes
```python
# Fixed configuration in model.scenic
height: 0.85  # DO NOT MODIFY - Full table for Scenic collision

# Hardcoded in simulator.py
'size': (obj.width, obj.length, 0.05)  # FIXED - MultiTableArena thickness only
```
