# test_simple_rotation.scenic
model scenic.simulators.robosuite.model

# Create table
table = new Table at (0.6, 0, 0.425)

# Test different rotations with boxes at different positions
box1 = new Box at (0.4, 0, 0.88),
    with color (1, 0, 0, 1),
    facing (0, 0, 0)  # No rotation

box2 = new Box at (0.6, 0, 0.88),
    with color (0, 1, 0, 1),
    facing (45 deg, 0, 0)  # 45 degree yaw

box3 = new Box at (0.8, 0, 0.88),
    with color (0, 0, 1, 1),
    facing (0, 30 deg, 0)  # 30 degree pitch

behavior CheckRotations():
    """Verify rotations are applied correctly."""
    print("Initial rotations (should match what was specified):")
    print(f"  Box1 (red): yaw={box1.yaw:.2f}, pitch={box1.pitch:.2f}, roll={box1.roll:.2f}")
    print(f"  Box2 (green): yaw={box2.yaw:.2f}, pitch={box2.pitch:.2f}, roll={box2.roll:.2f}")
    print(f"  Box3 (blue): yaw={box3.yaw:.2f}, pitch={box3.pitch:.2f}, roll={box3.roll:.2f}")
    
    for _ in range(5):
        wait
    
    print("\nAfter 5 steps (should remain the same):")
    print(f"  Box1 (red): yaw={box1.yaw:.2f}, pitch={box1.pitch:.2f}, roll={box1.roll:.2f}")
    print(f"  Box2 (green): yaw={box2.yaw:.2f}, pitch={box2.pitch:.2f}, roll={box2.roll:.2f}")
    print(f"  Box3 (blue): yaw={box3.yaw:.2f}, pitch={box3.pitch:.2f}, roll={box3.roll:.2f}")
    
    terminate simulation

ego = new Panda on (0, 0, 0),
    with behavior CheckRotations()