param map = localPath('../../../tests/formats/opendrive/maps/CARLA/Town05.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town05'
model scenic.simulators.carla.model #located in scenic/simulators/carla/model.scenic

ego = Car on lane, 
		facing Range(-10, 10) deg relative to roadDirection,
		with viewAngle 135 deg,
		with visibleDistance 30
car1 = Car ahead of ego by Range(3,5),
		facing roadDirection
ped1 = Bicycle on visible road, 
		apparently facing Range(-10,10) deg from car1,
		with regionContainedIn roadRegion
car2 = Car behind ego by Range(4, 10),
		facing roadDirection
car3 = Car left of ego by Range(4,6), 
		facing toward car1
Cone right of car3 by Range(2,3)
Bicycle beyond car1 by Range(-1,1) @ Range(2,5),
	with regionContainedIn roadRegion 
mc = Motorcycle offset along Range(-20, 20) deg by 0 @ Range(5,10),
		facing Range(-20,20) deg relative to roadDirection
mc2 = Motorcycle at (mc offset by Range(-2,2) @ Range(3,5)),
		with regionContainedIn roadRegion
Trash left of mc2 by Normal(1,1),
	with regionContainedIn roadRegion

require (distance from ego to mc2) < 30