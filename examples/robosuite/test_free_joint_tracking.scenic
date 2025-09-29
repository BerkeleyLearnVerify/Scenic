# test_free_joint_tracking.scenic
model scenic.simulators.robosuite.model

# Table at standard height
table = new Table at (0.6, 0, 0.425)

# Box that will fall and tumble
falling_box = new Box at (0.6, 0, 1.2),
    with color (1, 0, 0, 1)

# Box on table (should stay still)
stable_box = new Box on table,
    with color (0, 1, 0, 1)

behavior TrackFreeBodies():
    """Track positions and velocities of free-joint objects."""
    print("Initial state:")
    print(f"  Falling box: pos={falling_box.position}, vel={falling_box.velocity}")
    print(f"  Stable box: pos={stable_box.position}, vel={stable_box.velocity}")
    
    for step in range(20):
        if step % 5 == 0:
            print(f"\nStep {step}:")
            print(f"  Falling: z={falling_box.position.z:.3f}, vel_z={falling_box.velocity.z:.3f}")
            print(f"  Stable: z={stable_box.position.z:.3f}, vel_z={stable_box.velocity.z:.3f}")
        wait
    
    # Verify tracking worked
    if falling_box.position.z < 1.0:
        print("\n✓ Free joint tracking working - falling box moved")
    else:
        print("\n✗ Free joint tracking failed - box didn't fall")
    
    terminate simulation

ego = new Panda on (0, 0, 0),
    with behavior TrackFreeBodies()