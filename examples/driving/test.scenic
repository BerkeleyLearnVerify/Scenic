param map = localPath('../../assets/maps/CARLA/Town05.xodr')
model scenic.domains.driving.model

select_road = Uniform(*network.roads)
select_lane = Uniform(*select_road.lanes)

ego = new Car at (10,-12, 0), on road, with regionContainedIn everywhere

def foo():
    print(f"EGO POSITION: {ego.position}")
    return True
require foo()

#require ego.z > 1
# x = 10, y = -12
# at (10,-12, 100),