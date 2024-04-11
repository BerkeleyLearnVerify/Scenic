# NOTE: add your world info path here
param worldInfoPath = "C:/Users/piegu/Scenic/src/scenic/simulators/airsim/more/tests/testCar"

model scenic.simulators.airsim.modelCar


for i in range(3):
    new Car at (Range(-10,10),Range(-10,10),Range(0,10))
