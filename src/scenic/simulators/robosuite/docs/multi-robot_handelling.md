## Multi-Robot Action Handling Limitation

### Current Limitation
The current implementation only supports single-robot action execution despite allowing multiple robots in the scene. The `pending_robot_action` is a single variable that gets overwritten if multiple robots take actions.

```python
# Current: Single action buffer
self.pending_robot_action = None  # Only stores one robot's action
```

### Why This Exists
- Originally developed and tested with single-robot environments
- Multi-robot scene support was added later for positioning/visualization
- Action handling wasn't updated to support concurrent control

### Proposed Solution for Future Implementation

```python
class RobosuiteSimulation(Simulation):
    def __init__(self, ...):
        # Change from single action to per-robot actions
        self.pending_robot_actions = {}  # Dict[robot_idx: action_array]
        self.robot_action_dims = {}      # Dict[robot_idx: action_dim]
        
    def setup(self):
        """Initialize per-robot action dimensions."""
        # ... existing setup ...
        
        # Track action dimension per robot
        for i, robot in enumerate(self.robosuite_env.robots):
            if hasattr(robot, 'controller') and hasattr(robot.controller, 'control_dim'):
                self.robot_action_dims[i] = robot.controller.control_dim
            else:
                self.robot_action_dims[i] = DEFAULT_ACTION_DIM
    
    def executeActions(self, allActions):
        """Clear all pending actions before parent executes."""
        self.pending_robot_actions.clear()
        super().executeActions(allActions)
    
    def step(self):
        """Step with multi-robot action support."""
        # Build concatenated action vector for all robots
        full_action = []
        for i in range(len(self.robots)):
            if i in self.pending_robot_actions:
                action = self.pending_robot_actions[i]
            else:
                action = np.zeros(self.robot_action_dims.get(i, DEFAULT_ACTION_DIM))
            full_action.extend(action)
        
        action_array = np.array(full_action) if full_action else np.zeros(sum(self.robot_action_dims.values()))
        
        # Pass concatenated actions to RoboSuite
        obs, reward, done, info = self.robosuite_env.step(action_array)
```

### Action Classes Update
```python
class SetJointPositions(Action):
    def applyTo(self, agent, sim):
        if agent in sim.robots:
            robot_idx = sim.robots.index(agent)
            # Store action for specific robot
            sim.pending_robot_actions[robot_idx] = np.array(self.positions)
```

### Testing Requirements
- Single robot scenarios must remain compatible
- Test with 2+ robots taking simultaneous actions
- Verify action dimensions match controller expectations
- Test mixed robot types (Panda + UR5e) with different action dims

### Known Complexities
- Different robots may have different action dimensions
- RoboSuite expects concatenated action vector in robot creation order
- Controller types affect action dimension (OSC=7, Joint Position varies)
