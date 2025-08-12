param map = localPath('../../assets/maps/CARLA/Town04.xodr')
model scenic.domains.driving.model

select_road = Uniform(*network.roads)
select_lane = Uniform(*select_road.lanes)
select_intersection = Uniform(*network.intersections)

ego = new Car on road

def foo():
    print(f"EGO POSITION: {ego.position}")
    print(f"ROAD DIRECTION: {roadDirection at ego.position}")
    print(f"ROAD ORIENTATION: {ego.road.orientation[ego.position]}")
    print(f"EGO ORIENTATION: {ego.parentOrientation}")
    return True
require foo()