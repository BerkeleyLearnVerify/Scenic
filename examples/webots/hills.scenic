
model scenic.simulators.webots.model

WIDTH = 10
LENGTH = 10

workspace = Workspace(RectangularRegion((0, 0), 0, WIDTH, LENGTH))

corner = Point at (-WIDTH/2, -LENGTH/2)

ego = WebotsObject at corner offset by (1, 1), facing -45 deg,
    with width 0.5,
    with length 0.7,
    with webotsType 'Rover',
    with rotationOffset 90 deg

hill1 = Hill in workspace, with width WIDTH/2, with length LENGTH/2, with height 1
hill2 = Hill in workspace, with width WIDTH/4, with length LENGTH/4, with height 0.5

require (distance to hill1) > WIDTH/2
require (distance to hill2) > WIDTH/4

Ground with width WIDTH, with length LENGTH,
    with terrain [hill1, hill2]

monitor terminateOnT:
    keyboard = simulation().supervisor.getKeyboard()
    keyboard.enable(20)
    print('Select the 3D window and press T to terminate the scenario and generate a new one.')
    while True:
        wait
        if keyboard.getKey() == ord('T'):
            terminate
