param map = localPath('../../assets/maps/CARLA/Town01.xodr')
param sumo_map = localPath('../../assets/maps/CARLA/Town01.net.xml')
model scenic.simulators.metadrive.model

select_road = Uniform(*network.roads)
select_lane = Uniform(*select_road.lanes)
ego = new Car on select_lane.centerline

right_sidewalk = network.laneGroupAt(ego)._sidewalk

behavior WalkForward():
    while True:
        take SetWalkingDirectionAction(self.heading)
        take SetWalkingSpeedAction(0.5)

behavior StopWalking():
    while True:
        take SetWalkingSpeedAction(0)

behavior WalkThenStop():
    do WalkForward() for 100 steps
    do StopWalking() for 6 steps


new Pedestrian on visible right_sidewalk, with behavior WalkThenStop
