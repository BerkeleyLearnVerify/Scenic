# examples/robosuite/make_osc_test.scenic
"""Test RoboSuite's make() function with OSC controller for Lift task."""


model scenic.simulators.robosuite.model

simulator RobosuiteSimulator(
    render=True,
    real_time=True,
    speed=1.0,
    use_environment="Lift"
)

# cube = new Cube at (0.1, -0.1, 0.82)

behavior WorkingLiftBehavior():
    print("[LIFT] Starting lift behavior")
    

    wait
    wait
    
    sim = simulation()
    

    print("\n[LIFT] Phase 1: Opening gripper...")
    for i in range(20):
        take OSCPositionAction(position_delta=[0, 0, 0], gripper=-1)
    print("[LIFT] Gripper opened")
    

    print("\n[LIFT] Phase 2: Moving above cube...")
    moved_above = False
    for step in range(100):
        obs = sim._current_obs
        if not obs:
            print(f"[LIFT] Step {step}: No observation!")
            wait
            continue
            
        if 'cube_pos' not in obs or 'robot0_eef_pos' not in obs:
            print(f"[LIFT] Step {step}: Missing data!")
            wait
            continue
            
        cube_pos = obs['cube_pos']
        eef_pos = obs['robot0_eef_pos']
        

        error = [
            cube_pos[0] - eef_pos[0],
            cube_pos[1] - eef_pos[1], 
            cube_pos[2] + 0.1 - eef_pos[2]
        ]
        

        norm = (error[0]**2 + error[1]**2 + error[2]**2)**0.5
        
        if step % 10 == 0:
            print(f"[LIFT] Step {step}: Error = {norm:.3f}, EEF = {eef_pos}, Cube = {cube_pos}")
        
        if norm < 0.02:
            print(f"[LIFT] Reached above position at step {step}")
            moved_above = True
            break
        

        delta = [max(-0.3, min(0.3, e * 3)) for e in error]
        take OSCPositionAction(position_delta=delta, gripper=-1)
    
    if not moved_above:
        print("[LIFT] Failed to reach above position!")
    

    print("\n[LIFT] Phase 3: Moving to grasp position...")
    moved_down = False
    for step in range(80):
        obs = sim._current_obs
        if not obs or 'cube_pos' not in obs or 'robot0_eef_pos' not in obs:
            wait
            continue
            
        cube_pos = obs['cube_pos']
        eef_pos = obs['robot0_eef_pos']
        

        error = [
            cube_pos[0] - eef_pos[0],
            cube_pos[1] - eef_pos[1],
            cube_pos[2] - 0.02 - eef_pos[2]
        ]
        
        norm = (error[0]**2 + error[1]**2 + error[2]**2)**0.5
        
        if step % 10 == 0:
            print(f"[LIFT] Step {step}: Error = {norm:.3f}")
        
        if norm < 0.01:
            print(f"[LIFT] Reached grasp position at step {step}")
            moved_down = True
            break
        
        delta = [max(-0.2, min(0.2, e * 3)) for e in error]
        take OSCPositionAction(position_delta=delta, gripper=-1)
    
    if not moved_down:
        print("[LIFT] Failed to reach grasp position!")
    

    print("\n[LIFT] Phase 4: Closing gripper...")
    for i in range(30):
        take OSCPositionAction(position_delta=[0, 0, 0], gripper=1)
    print("[LIFT] Gripper closed")
    

    print("\n[LIFT] Phase 5: Lifting...")
    lifted = False
    initial_cube_height = None
    for step in range(100):
        obs = sim._current_obs
        if not obs or 'cube_pos' not in obs or 'robot0_eef_pos' not in obs:
            wait
            continue
            
        cube_pos = obs['cube_pos']
        eef_pos = obs['robot0_eef_pos']
        

        if initial_cube_height is None:
            initial_cube_height = cube_pos[2]
            print(f"[LIFT] Initial cube height: {initial_cube_height:.3f}")
        

        error = [
            cube_pos[0] - eef_pos[0],
            cube_pos[1] - eef_pos[1],
            cube_pos[2] + 0.15 - eef_pos[2]
        ]
        
        norm = (error[0]**2 + error[1]**2 + error[2]**2)**0.5
        
        if step % 10 == 0:
            cube_lift = cube_pos[2] - initial_cube_height
            print(f"[LIFT] Step {step}: Error = {norm:.3f}, Cube lifted by {cube_lift:.3f}m")
        
        if norm < 0.02:
            print(f"[LIFT] Reached lift position at step {step}")
            lifted = True
            break
        
        delta = [max(-0.3, min(0.3, e * 3)) for e in error]
        take OSCPositionAction(position_delta=delta, gripper=1)
    
    if not lifted:
        print("[LIFT] Failed to reach lift position!")
    

    print("\n[LIFT] Phase 6: Holding position...")
    for i in range(100):
        if i % 20 == 0:
            obs = sim._current_obs
            if obs and 'cube_pos' in obs and initial_cube_height is not None:
                cube_lift = obs['cube_pos'][2] - initial_cube_height
                print(f"[LIFT] Holding step {i}/100, cube height delta: {cube_lift:.3f}m")
        take OSCPositionAction(position_delta=[0, 0, 0], gripper=1)
    

    obs = sim._current_obs
    if obs and 'cube_pos' in obs and initial_cube_height is not None:
        final_cube_height = obs['cube_pos'][2]
        total_lift = final_cube_height - initial_cube_height
        success = total_lift > 0.05  # Success if lifted more than 5cm
        print(f"\n[LIFT] Final cube lift: {total_lift:.3f}m")
        print(f"[LIFT] Task Success: {success}")
    
    print(f"\n[LIFT] RoboSuite Success Check: {sim.checkSuccess()}")
    print("[LIFT] Behavior complete, continuing...")
    

    step_count = 0
    while True:
        wait
        step_count += 1
        if step_count % 100 == 0:
            print(f"[LIFT] Still running... (step {step_count})")

ego = new PandaRobot with behavior WorkingLiftBehavior()

terminate after 150 seconds  