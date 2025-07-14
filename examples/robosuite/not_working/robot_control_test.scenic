"""Basic robot control test - Joint position control"""

model scenic.simulators.robosuite.model

# Import action classes
from scenic.simulators.robosuite.simulator import SetJointPositionsAction, SetGripperAction

# Create a Panda robot with joint position controller
panda = new Panda with controller_type "JOINT_POSITION"

# Control behavior - move joints
behavior MoveJoints():
    # Initial wait to see starting position
    print("Starting robot control test...")
    for i in range(5):
        wait
    
    # Move to position 1
    print("Moving to position 1...")
    # Panda has 7 DOF, normalized values between -1 and 1
    # These values will be scaled to actual joint limits
    target_positions = [0.0, -0.3, 0.0, -0.5, 0.0, 0.5, 0.0]
    take SetJointPositionsAction(target_positions)
    
    # Wait to see the movement
    for i in range(20):
        wait
    
    # Move to position 2
    print("Moving to position 2...")
    target_positions = [0.3, 0.0, -0.3, -0.3, 0.3, 0.3, -0.3]
    take SetJointPositionsAction(target_positions)
    
    # Wait to see the movement
    for i in range(20):
        wait
    
    # Move to position 3 with gripper action
    print("Moving to position 3 and closing gripper...")
    target_positions = [-0.3, -0.2, 0.3, -0.6, -0.3, 0.6, 0.3]
    take SetJointPositionsAction(target_positions), SetGripperAction(1.0)  # Close gripper
    
    for i in range(20):
        wait
    
    # Open gripper
    print("Opening gripper...")
    take SetGripperAction(-1.0)  # Open gripper
    
    for i in range(10):
        wait
    
    print("Robot control test completed!")

panda.behavior = MoveJoints()

# Monitor robot state
monitor RobotMonitor():
    for i in range(80):  # Run for entire duration
        if i % 5 == 0:  # Print every 5 steps to reduce clutter
            print(f"Step {i}:")
            if panda.joint_positions is not None:
                print(f"  Joint positions: {[f'{p:.3f}' for p in panda.joint_positions]}")
            else:
                print(f"  Joint positions: None")
            print(f"  EE position: {panda.ee_position}")
            if panda.gripper_state is not None:
                print(f"  Gripper state: {panda.gripper_state:.3f}")
        wait

require monitor RobotMonitor()

terminate after 80 steps