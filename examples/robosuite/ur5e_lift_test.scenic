# examples/robosuite/ur5e_lift_test.scenic
"""Test UR5e robot cube pickup with optimized parameters."""

param use_environment = "Lift"

model scenic.simulators.robosuite.model

simulator RobosuiteSimulator(
    render=True,
    real_time=True,
    speed=1.0,
    use_environment=globalParameters.use_environment
)

# Define UR5e robot class
class UR5eRobot(Robot):
    """UR5e robot configuration."""
    robot_type: "UR5e"
    initial_qpos: None  # Use default

behavior UR5eLiftBehavior():
    """Optimized lift behavior for UR5e robot."""
    
    print("=== UR5e OPTIMIZED CUBE PICKUP ===")
    
    # Wait for stabilization
    wait
    wait
    
    sim = simulation()
    
    # Phase 1: Open gripper (20 steps)
    print("\nOpening gripper...")
    for i in range(20):
        take OSCPositionAction(position_delta=[0, 0, 0], gripper=-1)
    
    # Phase 2: Move above cube (40 steps max)
    print("\n1. Moving above cube...")
    for step in range(40):
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
        
        if norm < 0.02:
            print(f"   Reached above position at step {step}")
            break
        
        # Scale and clip
        delta = [max(-0.3, min(0.3, e * 3)) for e in error]
        take OSCPositionAction(position_delta=delta, gripper=-1)
    
    # Phase 3: Move down to cube (30 steps max)
    print("2. Moving to grasp position...")
    for step in range(30):
        obs = sim._current_obs
        if not obs or 'cube_pos' not in obs or 'robot0_eef_pos' not in obs:
            wait
            continue
            
        cube_pos = obs['cube_pos']
        eef_pos = obs['robot0_eef_pos']
        
        # Target is 2.5cm below cube center
        error = [
            cube_pos[0] - eef_pos[0],
            cube_pos[1] - eef_pos[1],
            cube_pos[2] - 0.025 - eef_pos[2]
        ]
        
        norm = (error[0]**2 + error[1]**2 + error[2]**2)**0.5
        
        if norm < 0.01:
            print(f"   Reached grasp position at step {step}")
            break
        
        # Slower approach
        delta = [max(-0.1, min(0.1, e * 2.0)) for e in error]
        take OSCPositionAction(position_delta=delta, gripper=-1)
    
    # Phase 4: Grasp (20 steps)
    print("3. Closing gripper...")
    for i in range(20):
        take OSCPositionAction(position_delta=[0, 0, 0], gripper=1)
    
    # Phase 5: Lift (50 steps max)
    print("4. Lifting...")
    initial_cube_height = None
    success_achieved = False
    
    for step in range(50):
        obs = sim._current_obs
        if not obs or 'cube_pos' not in obs or 'robot0_eef_pos' not in obs:
            wait
            continue
            
        cube_pos = obs['cube_pos']
        eef_pos = obs['robot0_eef_pos']
        
        # Track initial height
        if initial_cube_height is None:
            initial_cube_height = cube_pos[2]
            print(f"   Initial cube height: {initial_cube_height:.3f}")
        
        # Target is 25cm above original position
        error = [
            cube_pos[0] - eef_pos[0],
            cube_pos[1] - eef_pos[1],
            cube_pos[2] + 0.25 - eef_pos[2]
        ]
        
        delta = [max(-0.25, min(0.25, e * 2.5)) for e in error]
        take OSCPositionAction(position_delta=delta, gripper=1)
        
        # Check success
        if sim.checkSuccess() and step > 10 and not success_achieved:
            print(f"   Success achieved at lift step {step}!")
            success_achieved = True
    
    # Phase 6: Hold position (30 steps)
    print("5. Holding position...")
    for i in range(30):
        if i % 10 == 0:
            obs = sim._current_obs
            if obs and 'cube_pos' in obs:
                print(f"   Cube height: {obs['cube_pos'][2]:.3f} (needs > 0.84)")
        take OSCPositionAction(position_delta=[0, 0, 0], gripper=1)
    
    # Final check
    obs = sim._current_obs
    if obs and 'cube_pos' in obs:
        print(f"\nFinal Success: {sim.checkSuccess()}")
        print(f"Cube height: {obs['cube_pos'][2]:.3f} (needs > 0.84)")
    
    print("\nBehavior complete, continuing...")
    
    # Keep running
    while True:
        try:
            wait
        except:
            print("\nEpisode terminated. Exiting...")
            break

ego = new UR5eRobot with behavior UR5eLiftBehavior()

terminate after 100 seconds  # Longer time for UR5e