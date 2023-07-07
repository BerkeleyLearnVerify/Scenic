
from scenic.simulators.webots.road.world import setLocalWorld
setLocalWorld(__file__, 'southside2.wbt')

from scenic.simulators.webots.road.model import *

depth = 4
laneGap = 3.5
carGap = Range(1, 3)
laneShift = Range(-2, 2)
wiggle = Range(-5 deg, 5 deg)

# Helper function for making platoons
def createPlatoonAt(car, numCars, model=None, dist=Range(2, 8),
                    shift=Range(-0.5, 0.5), wiggle=0):
    cars = [car]
    lastCar = car
    for i in range(numCars-1):
        center = follow roadDirection from (front of lastCar) for resample(dist)
        pos = new OrientedPoint right of center by shift, facing resample(wiggle) relative to roadDirection
        lastCar = new Car ahead of pos, with model (car.model if model is None else resample(model))
        cars.append(lastCar)
    return cars

def carAheadOfCar(car, gap, offsetX=0, wiggle=0):
    pos = new OrientedPoint at (front of car) offset by (offsetX @ gap), \
        facing resample(wiggle) relative to roadDirection
    return new Car ahead of pos

ego = new Car with visibleDistance 60
modelDist = modelWithName['ToyotaPrius']

leftCar = carAheadOfCar(ego, laneShift + carGap, offsetX=-laneGap, wiggle=wiggle)
createPlatoonAt(leftCar, depth, dist=carGap, wiggle=wiggle, model=modelDist)

midCar = carAheadOfCar(ego, resample(carGap), wiggle=wiggle)
createPlatoonAt(midCar, depth, dist=carGap, wiggle=wiggle, model=modelDist)

rightCar = carAheadOfCar(ego, resample(laneShift) + resample(carGap),
    offsetX=laneGap, wiggle=wiggle)
createPlatoonAt(rightCar, depth, dist=carGap, wiggle=wiggle, model=modelDist)
