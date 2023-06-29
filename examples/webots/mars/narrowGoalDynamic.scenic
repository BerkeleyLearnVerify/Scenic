model scenic.simulators.webots.model
from mars_lib import *

# Ground with random gaussian hills
ground = new MarsGround on (0,0,0), with terrain [new MarsHill for _ in range(60)]

# Ego and goal on ground
ego = new Rover at (0, -3), on ground, with controller 'sojourner'
goal = new Goal at (Range(-2, 2), Range(2, 3)), on ground, facing (0,0,0)

# Bottleneck made of two pipes with a rock in between
bottleneck = new OrientedPoint at ego offset by Range(-1.5, 1.5) @ Range(0.5, 1.5), facing Range(-30, 30) deg
require abs((angle to goal) - (angle to bottleneck)) <= 10 deg
new BigRock at bottleneck, on ground

gap = 1.2 * ego.width
halfGap = gap / 2

leftEdge = new OrientedPoint left of bottleneck by halfGap,
    facing Range(60, 120) deg relative to bottleneck.heading
rightEdge = new OrientedPoint right of bottleneck by halfGap,
    facing Range(-120, -60) deg relative to bottleneck.heading

new Pipe ahead of leftEdge, with length Range(1, 2), on ground, facing leftEdge, with parentOrientation 0
new Pipe ahead of rightEdge, with length Range(1, 2), on ground, facing rightEdge, with parentOrientation 0

# Other junk because why not?

new Pipe on ground, with parentOrientation 0
new BigRock beyond bottleneck by Range(0.25, 0.75) @ Range(0.75, 1), on ground
new BigRock beyond bottleneck by Range(-0.75, -0.25) @ Range(0.75, 1), on ground
new Rock on ground
new Rock on ground
new Rock on ground

# Dynamic components
monitor terminateOnT():
    keyboard = simulation().supervisor.getKeyboard()
    keyboard.enable(20)
    print('Select the 3D window and press T to terminate the scenario and generate a new one.')
    while True:
        wait
        if keyboard.getKey() == ord('T'):
            terminate

require monitor terminateOnT()

terminate when (distance to goal) <= 0.2
