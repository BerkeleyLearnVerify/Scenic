param map = localPath('../../assets/maps/CARLA/Town01.xodr')

model scenic.simulators.metadrive.model

# behavior WalkForward():
#     while True:
#         take SetWalkingDirectionAction(self.heading), SetWalkingSpeedAction(0.5)

# behavior StopWalking():
#     while True:
#         take SetWalkingSpeedAction(0)

# behavior WalkThenStop():
#     do WalkForward() for 2 steps
#     do StopWalking() for 2 steps

# ego = new Car at (30, 2)
# pedestrian = new Pedestrian at (50, 6), with behavior WalkThenStop


scenario TrailingCar(gap=0.5):
    setup:
        trailingCar = new Car 

scenario Main():
    setup:
        # ego = new Car with behavior FollowLaneBehavior(target_speed=5)
        tc = TrailingCar(gap=0.5)
    compose:
        do tc

Main()
