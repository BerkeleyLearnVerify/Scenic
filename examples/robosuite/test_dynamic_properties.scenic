# test_simple_dynamics.scenic
model scenic.simulators.robosuite.model

param render = True
param real_time = False

# Single robot
behavior MonitorProperties():
    """Monitor dynamic properties."""
    for step in range(30):
        print(f"\nStep {step}:")
        print(f"  Joint positions: {self.jointPositions[:3]}")
        print(f"  EEF position: {self.eefPos}")
        print(f"  Gripper: {self.gripperState}")
        
        # Simple movement
        if step < 10:
            take OSCPositionAction(position_delta=[0, 0, 0.01])  # Up
        elif step < 20:
            take OSCPositionAction(position_delta=[0.01, 0, 0])  # Forward
        else:
            take OSCPositionAction(gripper=1)  # Close gripper

    terminate simulation

ego = new Panda on (0, 0, 0),
    with behavior MonitorProperties()