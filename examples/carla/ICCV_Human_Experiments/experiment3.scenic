model scenic.simulators.carla.model

offset = Uniform(-1,1) * Range(90, 180) deg

ego = Car on drivableRoad,
		facing offset relative to roadDirection,
		with visibleDistance 50,
		with viewAngle 135 deg

otherCar = Car on visible road,
			facing Range(-15, 15) deg relative to roadDirection

require (distance from ego to otherCar) < 10