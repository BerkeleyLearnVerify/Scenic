# examples/robosuite/lift_custom_position.scenic
"""Lift task with custom cube position - demonstrates object positioning."""

param use_environment = "Lift"
model scenic.simulators.robosuite.model

# Define cube class for Lift environment
class LiftCube(RoboSuiteObject):
    """Cube in Lift environment."""
    _env_object_name: "cube"  # Maps to RoboSuite's internal cube
    width: 0.05
    length: 0.05    
    height: 0.05
    color: (1, 0, 0)  # Red

# Create cube at custom position
# Default Lift places cube at (0, 0, ~0.82)
# Let's place it to the right and forward
cube = new LiftCube at (0.1, -0.1, 0.82)

# Robot with modified lift behavior for new position
behavior CustomPositionLiftBehavior():
    print("=== LIFT WITH CUSTOM CUBE POSITION ===")
    
    # Wait for stabilization
    wait
    wait
    
    sim = simulation()
    
    # Print initial cube position
    obs = sim._current_obs
    if obs and 'cube_pos' in obs:
        print(f"Initial cube position: {obs['cube_pos']}")
    
    # Phase 1: Open gripper
    print("\nPhase 1: Opening gripper...")
    for i in range(20):
        take OSCPositionAction(position_delta=[0, 0, 0], gripper=-1)
    
    # Phase 2: Move above cube
    print("\nPhase 2: Moving above cube...")
    for step in range(60):
        obs = sim._current_obs
        if not obs or 'cube_pos' not in obs or 'robot0_eef_pos' not in obs:
            wait
            continue
            
        cube_pos = obs['cube_pos']
        eef_pos = obs['robot0_eef_pos']
        
        # Target is 10cm above cube
        error = [
            cube_pos[0] - eef_pos[0],
            cube_pos[1] - eef_pos[1], 
            cube_pos[2] + 0.1 - eef_pos[2]
        ]
        
        norm = (error[0]**2 + error[1]**2 + error[2]**2)**0.5
        
        if step % 10 == 0:
            print(f"  Step {step}: Moving to cube at ({cube_pos[0]:.3f}, {cube_pos[1]:.3f}, {cube_pos[2]:.3f})")
        
        if norm < 0.02:
            print(f"  Reached above position at step {step}")
            break
        
        delta = [max(-0.3, min(0.3, e * 3)) for e in error]
        take OSCPositionAction(position_delta=delta, gripper=-1)
    
    # Phase 3: Move down to cube
    print("\nPhase 3: Moving to grasp position...")
    for step in range(40):
        obs = sim._current_obs
        if not obs or 'cube_pos' not in obs or 'robot0_eef_pos' not in obs:
            wait
            continue
            
        cube_pos = obs['cube_pos']
        eef_pos = obs['robot0_eef_pos']
        
        # Target is at cube center
        error = [
            cube_pos[0] - eef_pos[0],
            cube_pos[1] - eef_pos[1],
            cube_pos[2] - 0.02 - eef_pos[2]
        ]
        
        norm = (error[0]**2 + error[1]**2 + error[2]**2)**0.5
        
        if norm < 0.01:
            print(f"  Reached grasp position at step {step}")
            break
        
        delta = [max(-0.2, min(0.2, e * 3)) for e in error]
        take OSCPositionAction(position_delta=delta, gripper=-1)
    
    # Phase 4: Close gripper
    print("\nPhase 4: Closing gripper...")
    for i in range(30):
        take OSCPositionAction(position_delta=[0, 0, 0], gripper=1)
    
    # Phase 5: Lift
    print("\nPhase 5: Lifting...")
    for step in range(60):
        obs = sim._current_obs
        if not obs or 'cube_pos' not in obs:
            wait
            continue
            
        cube_height = obs['cube_pos'][2]
        
        if step % 10 == 0:
            print(f"  Step {step}: Cube height = {cube_height:.3f}")
        
        # Lift up
        take OSCPositionAction(position_delta=[0, 0, 0.3], gripper=1)
        
        if cube_height > 0.9:  # Success threshold
            print(f"  Lifted successfully at step {step}!")
            break
    
    # Phase 6: Hold
    print("\nPhase 6: Holding position...")
    for i in range(50):
        take OSCPositionAction(position_delta=[0, 0, 0], gripper=1)
    
    # Check success
    success = sim.checkSuccess()
    print(f"\nTask Success: {success}")
    
    print("\nBehavior complete, continuing...")
    while True:
        wait

ego = new PandaRobot with behavior CustomPositionLiftBehavior()

# Monitor to show cube position
monitor CubePositionMonitor():
    print("\n=== Cube Position Monitor ===")
    sim = simulation()
    step = 0
    
    while True:
        obs = sim._current_obs
        if obs and 'cube_pos' in obs:
            if step % 20 == 0:
                pos = obs['cube_pos']
                print(f"[Step {step:3d}] Cube at: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
        step += 1
        wait

require monitor CubePositionMonitor()

terminate after 80 seconds