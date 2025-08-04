# examples/robosuite/cube_random_position.scenic
"""Randomized cube position with bounded lift behavior."""

model scenic.simulators.robosuite.model

param use_environment = "Lift"
param render = True
param real_time = False  # Override default: run as fast as possible
param camera_view = "birdview"

# Use parameters from above
simulator RobosuiteSimulator(
    render=globalParameters.render,
    real_time=globalParameters.real_time,
    use_environment=globalParameters.use_environment,
    camera_view=globalParameters.camera_view
)

# SCENIC RANDOMIZATION: cube position randomly sampled each run
cube = new LiftCube at (Range(-0.25, 0.25), Range(-0.25, 0.25), 0.82)

behavior AdaptiveLiftBehavior():
    """Fast pickup behavior with bounded lift."""
    
    wait; wait  # Initial stabilization only
    
    sim = simulation()
    
    # Get initial cube position
    obs = sim.getCurrentObservation()
    if obs and 'cube_pos' in obs:
        print(f"\n=== RANDOMIZED CUBE PICKUP ===")
        print(f"Cube spawned at: ({obs['cube_pos'][0]:.3f}, {obs['cube_pos'][1]:.3f}, {obs['cube_pos'][2]:.3f})")
    
    # Phase 1: Open gripper (20 steps)
    print("\nOpening gripper...")
    for i in range(20):
        take OSCPositionAction(position_delta=[0, 0, 0], gripper=-1)
    
    # Phase 2: Move above cube
    print("Moving above cube...")
    for step in range(100):
        obs = sim.getCurrentObservation()
        if not obs or 'cube_pos' not in obs or 'robot0_eef_pos' not in obs:
            wait; continue
            
        cube_pos = obs['cube_pos']
        eef_pos = obs['robot0_eef_pos']
        
        error = [
            cube_pos[0] - eef_pos[0],
            cube_pos[1] - eef_pos[1], 
            cube_pos[2] + 0.1 - eef_pos[2]
        ]
        
        if (error[0]**2 + error[1]**2 + error[2]**2)**0.5 < 0.02:
            print(f"  Reached above position at step {step}")
            break
        
        delta = [max(-0.3, min(0.3, e * 3)) for e in error]
        take OSCPositionAction(position_delta=delta, gripper=-1)
    
    # Phase 3: Move down to grasp
    print("Moving to grasp position...")
    for step in range(80):
        obs = sim.getCurrentObservation()
        if not obs or 'cube_pos' not in obs or 'robot0_eef_pos' not in obs:
            wait; continue
            
        cube_pos = obs['cube_pos']
        eef_pos = obs['robot0_eef_pos']
        
        error = [
            cube_pos[0] - eef_pos[0],
            cube_pos[1] - eef_pos[1],
            cube_pos[2] - 0.02 - eef_pos[2]
        ]
        
        if (error[0]**2 + error[1]**2 + error[2]**2)**0.5 < 0.01:
            print(f"  Reached grasp position at step {step}")
            break
        
        delta = [max(-0.2, min(0.2, e * 3)) for e in error]
        take OSCPositionAction(position_delta=delta, gripper=-1)
    
    # Phase 4: Close gripper (minimal steps)
    print("Closing gripper...")
    take OSCPositionAction(position_delta=[0, 0, 0], gripper=1)
    take OSCPositionAction(position_delta=[0, 0, 0], gripper=1)
    take OSCPositionAction(position_delta=[0, 0, 0], gripper=1)
    
    # Phase 5: Lift with hard boundary
    print("Lifting...")
    TARGET_HEIGHT = 1.0  # RoboSuite success threshold
    MAX_LIFT_HEIGHT = 1.0  # Hard boundary
    
    for step in range(100):
        obs = sim.getCurrentObservation()
        if not obs or 'cube_pos' not in obs or 'robot0_eef_pos' not in obs:
            wait; continue
            
        cube_pos = obs['cube_pos']
        eef_pos = obs['robot0_eef_pos']
        
        # Success check
        if cube_pos[2] > TARGET_HEIGHT:
            success = sim.checkSuccess()
            print(f"\nSUCCESS: {success}")
            print(f"Cube height: {cube_pos[2]:.3f}m (> {TARGET_HEIGHT}m)")
            print(f"Completed in {step} lift steps")
            terminate simulation
        
        # Safety check
        if cube_pos[2] > MAX_LIFT_HEIGHT:
            print(f"\nMax height reached: {cube_pos[2]:.3f}m")
            terminate simulation
        
        # Target: 15cm above initial
        error = [
            cube_pos[0] - eef_pos[0],
            cube_pos[1] - eef_pos[1],
            cube_pos[2] + 0.15 - eef_pos[2]
        ]
        
        delta = [max(-0.3, min(0.3, e * 3)) for e in error]
        take OSCPositionAction(position_delta=delta, gripper=1)
    
    print("Failed to reach target height")
    terminate simulation

# ego = new PandaRobot with behavior AdaptiveLiftBehavior()
ego = new UR5eRobot with behavior AdaptiveLiftBehavior()