param map = localPath('../../assets/maps/CARLA/Town05.xodr')
model scenic.domains.driving.model

select_road = Uniform(*network.roads)
select_lane = Uniform(*select_road.lanes)

ego = new Car with regionContainedIn everywhere
#ego.parentOrientation = ego.road.orientation[ego.position]

def foo():
    print(f"EGO POSITION: {ego.position}")
    print(f"EGO ORIENTATION: {ego.parentOrientation}")
    print(f"EGO ROAD: {network.roadAt(ego.position)}")
    return True
require foo()

require ego.z > 8
require ego.x > 5
require ego.x < 15
# x = 10, y = -12
# at (10,-12, 100),
#"""at (10,-12, 11.493139700648731)"""
#, on road, 