# NOTE: add your world info path here
# param worldInfoPath = "[YOUR PATH HERE]"

model scenic.simulators.airsim.model

behavior FlyForwardsSafely(speed = 5,tolerance = 5):
    client = simulation().client

    while True:
        do MoveByVelocity((0,4,0),1)
        frontDistance = client.getDistanceSensorData(vehicle_name=self.realObjName,distance_sensor_name="frontDistance").distance
        if frontDistance < tolerance:
            return

    return


new Drone at (0,10,0),
    with behavior FlyForwardsSafely()
