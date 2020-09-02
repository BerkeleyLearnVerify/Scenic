from scenic.simulators.gta.map import setLocalMap
setLocalMap(__file__, 'map.npz')

from scenic.simulators.gta.model import *

depth = 2
laneGap = 3.5
carGap = Range(10, 20)
laneShift = Range(-10, 10)
wiggle = Range(-5 deg, 5 deg)

def carAheadOfCar(car, gap, offsetX=0, wiggle=0):
	pos = OrientedPoint at (front of car) offset by (offsetX @ gap),
		facing resample(wiggle) relative to roadDirection
	return Car ahead of pos

ego = Car with visibleDistance 100
modelDist = CarModel.defaultModel()

leftCar = carAheadOfCar(ego, laneShift + carGap, offsetX=-laneGap, wiggle=wiggle)
createPlatoonAt(leftCar, depth, dist=carGap, wiggle=wiggle, model=modelDist)

rightCar = carAheadOfCar(ego, resample(laneShift) + resample(carGap),
	offsetX=laneGap, wiggle=wiggle)
createPlatoonAt(rightCar, depth, dist=carGap, wiggle=wiggle, model=modelDist)