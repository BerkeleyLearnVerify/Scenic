# test_velocity_debug.scenic
model scenic.simulators.robosuite.model

falling_box = new Box at (0.6, 0, 1.2),
    with color (1, 0, 0, 1)

behavior DebugVelocity():
    prev_pos = falling_box.position
    for step in range(10):
        curr_pos = falling_box.position
        # Manual velocity calculation
        manual_vel = (curr_pos.z - prev_pos.z) / 0.01  # assuming 0.01s timestep
        print(f"Step {step}: z={curr_pos.z:.3f}, vel_from_sim={falling_box.velocity.z:.3f}, manual_vel={manual_vel:.3f}")
        prev_pos = curr_pos
        wait
    terminate simulation

ego = new Panda on (0, 0, 0),
    with behavior DebugVelocity()