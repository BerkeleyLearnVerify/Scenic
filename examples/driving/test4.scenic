param map = localPath('../../assets/maps/CARLA/Town04.xodr')
model scenic.domains.driving.model

ego = new Car on road

def foo():
    print(f"EGO POSITION: {ego.position}")
    print(f"EGO ORIENTATION: {ego.parentOrientation}")
    print(f"ROAD DIRECTION: {roadDirection at ego.position}")
    print(f"ROAD ORIENTATION: {ego.road.orientation[ego.position]}")
    return True
require foo()