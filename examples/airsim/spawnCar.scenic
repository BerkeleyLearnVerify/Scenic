# NOTE: add your world info path here
param worldInfoPath = "C:/Users/piegu/Scenic/examples/airsim/testCar"

model scenic.simulators.airsim.modelCar


for i in range(3):
    new Car at (Range(-10,10),Range(-10,10),Range(0,10))
