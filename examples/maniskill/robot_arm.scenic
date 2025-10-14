model scenic.simulators.maniskill.model

import math
pi = math.pi

# Create ground plane
ground = new Ground with position (0, 0, -0.920)

# Create Panda robot arm with a neutral pose
# Joint angles: [joint1, joint2, joint3, joint4, joint5, joint6, joint7, gripper_left, gripper_right]
panda = new Robot,
    with uuid "panda",
    with jointAngles [0.0, pi/8, 0.0, -pi/2, 0.0, pi*5/8, pi/4, 0.04, 0.04]

# Create camera positioned to view the robot and cube
camera = new Camera,
    at (-0.64, -0.4, 0.758),
    with name "base_camera",
    with img_width 640,
    with img_height 480,
    with fov 1.0

# Create a cube in front of the robot (within reach)
cube = new ManiskillObject on ground,
    at (0.4, 0.0, 0.035),
    with name "target_cube",
    with shape BoxShape(),
    with width 0.05,
    with length 0.05,
    with height 0.05,
    with color [1.0, 0.5, 0.0, 1.0]  # Orange cube
