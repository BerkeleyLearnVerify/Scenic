# param map_path = localPath('C:/Users/Mary/Documents/Code/Scenic/AirSimBinaries/Blocks/WindowsNoEditor/Blocks.exe')
model scenic.simulators.airsim.model


# ego = new Drone at (0,0,10),
#     with behavior MoveToPosition(4,4,2)

behavior flyAndThenFlyToHome():
    do FlyToPosition((10,10,10))
    do FlyToStart()
    
# highDrone = new Drone at (3,3,3)
# new Drone at (-3,-3,3)

# ego = new Drone at (5,5,5), with behavior Patrol([(-1,2,2),(1,4,2),(-1,4,2),(-1,2,4),(1,2,4),(1,4,4),(-1,4,4)],True)
# new Drone at (8,8,8), with behavior Follow(ego)

new Drone at (3,3,3), with behavior FlyToPosition((0,0,0))
new Drone at (-3,-3,3), with behavior FlyToPosition((0,0,0))

new StaticObj at (0,10,0),
    with width 2,
    with assetName "Cone",
    with name "fun"

new StaticObj at (0,20,0),
    with width 2,
    with assetName "Cone",
    with name "cool"
# ego = new Drone at (0,0,0)

# record ego.position as "ego position"