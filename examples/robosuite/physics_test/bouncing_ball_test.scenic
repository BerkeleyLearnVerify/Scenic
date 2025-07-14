"""Bouncing Ball Test - Proper monitoring approach"""

model scenic.simulators.robosuite.model

class Ball(Object):
    width: 0.2  # Larger ball
    length: 0.2
    height: 0.2
    color: (1, 0, 0)

# Drop ball from higher
ball = new Ball at (0, 0, 2.0)

# Monitor using Scenic's built-in properties
monitor BounceMonitor():
    print("=== Ball Drop Monitor ===")
    prev_vz = 0
    
    for i in range(50):
        z = ball.position[2]
        vz = ball.velocity[2]
        speed = ball.speed
        
        print(f"Step {i:2d}: z={z:.3f}m, vz={vz:.3f}m/s, speed={speed:.3f}m/s")
        
        # Detect bounce (velocity changes from negative to positive)
        if i > 0 and prev_vz < -0.1 and vz > 0.1:
            print(f"  *** BOUNCE DETECTED at z={z:.3f}m ***")
        
        prev_vz = vz
        wait
    
    terminate

require monitor BounceMonitor()

# Record data for analysis
record ball.position[2] as height
record ball.velocity[2] as vertical_velocity
record ball.speed as speed

terminate after 50 steps