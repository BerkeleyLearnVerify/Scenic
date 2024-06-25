# param use_table = False
# table = new Table at (20, 20, 20)


# examples/robosuite/tower_collapse.scenic
"""Tower Collapse Simulation - Fixed Version
Stops when tower falls (3+ cylinders down) or stabilizes.
"""

model scenic.simulators.robosuite.model

class Cylinder(Object):
    width: 0.08
    length: 0.08  
    height: 0.12
    color: Uniform((1,0,0), (0,1,0), (0,0,1), (1,1,0), (1,0,1))

# Build tower with random offsets for instability
cylinder1 = new Cylinder at (Range(-0.01, 0.01), Range(-0.01, 0.01), 0.10)
cylinder2 = new Cylinder at (Range(-0.01, 0.01), Range(-0.01, 0.01), 0.221)
cylinder3 = new Cylinder at (Range(-0.01, 0.01), Range(-0.01, 0.01), 0.342)
cylinder4 = new Cylinder at (Range(-0.01, 0.01), Range(-0.01, 0.01), 0.463)
cylinder5 = new Cylinder at (Range(-0.01, 0.01), Range(-0.01, 0.01), 0.584)
cylinder6 = new Cylinder at (Range(-0.01, 0.01), Range(-0.01, 0.01), 0.705)

tower_cylinders = [cylinder1, cylinder2, cylinder3, cylinder4, cylinder5, cylinder6]

# Set ego to bottom cylinder
ego = cylinder1

# Records
record [c.position[0] for c in tower_cylinders] as cylinder_x_positions
record [c.position[2] for c in tower_cylinders] as cylinder_heights  
record max([abs(c.position[0]) for c in tower_cylinders]) as tower_sway
record len([c for c in tower_cylinders if c.position[2] < 0.05]) as fallen_count

# Monitor for collapse and stability
monitor TowerMonitor():
    print(f"=== Tower Collapse Monitor Started ===")
    step_count = 0
    stable_steps = 0
    prev_positions = None
    
    while True:
        fallen = len([c for c in tower_cylinders if c.position[2] < 0.05])
        sway = max([abs(c.position[0]) for c in tower_cylinders])
        
        # Check stability - if positions haven't changed much
        current_positions = [(c.position[0], c.position[2]) for c in tower_cylinders]
        if prev_positions:
            max_change = max([abs(curr[0]-prev[0]) + abs(curr[1]-prev[1]) 
                            for curr, prev in zip(current_positions, prev_positions)])
            if max_change < 0.001:  # Very little movement
                stable_steps += 1
            else:
                stable_steps = 0
        prev_positions = current_positions
        
        # Print status every 10 steps
        if step_count % 10 == 0:
            print(f"[Step {step_count:3d}] Fallen={fallen}/6, Sway={sway:.3f}, Stable={stable_steps}")
            if step_count % 30 == 0:  # Detailed print every 30 steps
                for i, c in enumerate(tower_cylinders):
                    print(f"  Cylinder{i+1}: x={c.position[0]:.3f}, z={c.position[2]:.3f}")
        
        # Terminate on collapse (3+ fallen)
        if fallen >= 3:
            print(f"\n!!! TOWER COLLAPSED at step {step_count}: {fallen}/6 cylinders down !!!")
            terminate
            
        # Terminate if stable for 20 steps
        if stable_steps >= 20:
            print(f"\n=== Tower STABILIZED at step {step_count} ===")
            terminate
            
        step_count += 1
        wait

require monitor TowerMonitor()

# Termination conditions
terminate when len([c for c in tower_cylinders if c.position[2] < 0.05]) >= 3   
terminate after 50 steps  # Max simulation length