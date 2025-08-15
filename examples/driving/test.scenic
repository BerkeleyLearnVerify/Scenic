param map = localPath('../../assets/maps/CARLA/Town04.xodr')
model scenic.domains.driving.model

select_road = Uniform(*network.roads)
select_lane = Uniform(*select_road.lanes)
select_intersection = Uniform(*network.intersections)

current_object = select_intersection

ego = new Car on current_object
#breakpoint()
#roadDirection at (10,-12,100)
#ego.parentOrientation = ego.road.orientation[ego.position]

def foo():
    print(f"EGO POSITION: {ego.position}")
    print(f"ROAD ID: {current_object.id}")
#    print(f"ROAD DIRECTION: {roadDirection at ego.position}")
    print(f"ROAD ORIENTATION: {ego.road.orientation[ego.position]}")
    print(f"SELECT ROAD ORIENTATION: {current_object.orientation[ego.position]}")
#    print(ego.elementAt(ego.position))
#    print(f"EGO ORIENTATION: {ego.parentOrientation}")
    print(f"Ego Road: {ego._road}")
    print(f"Ego Intersection: {ego._intersection}")
    return True
require foo()
# with regionContainedIn everywhere
# x = 10, y = -12
# at (10,-12, 100),
#"""at (10,-12, 11.493139700648731)"""
#, on road, 