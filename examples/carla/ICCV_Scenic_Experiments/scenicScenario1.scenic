param map = localPath('../../../tests/formats/opendrive/maps/CARLA/Town01.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town01'
model scenic.simulators.carla.model #located in scenic/simulators/carla/model.scenic

def createPlatoonAt(car, numCars, model=None, dist=Range(2, 8),
                    shift=Range(-0.5, 0.5), wiggle=0):
    """Create a platoon starting from the given car."""
    lastCar = car
    for i in range(numCars):
        center = follow roadDirection from (front of lastCar) for resample(dist)
        pos = OrientedPoint right of center by shift, facing resample(wiggle) relative to roadDirection
        lastCar = Car ahead of pos

ego = Car with visibleDistance 60
c2 = Car visible
platoon = createPlatoonAt(c2, 1, dist=Range(2, 5))

