param map = localPath('../../assets/maps/CARLA/Town05.xodr')
model scenic.simulators.metadrive.model



# behavior TeleportForward():
#     for x in range(20):
#         wait
#     take SetPositionAction(Vector(50, 50))

# ego = new Car at (0,0), with behavior TeleportForward()



# behavior ForwardThenStopThenReverse():
#     # Drive forward for 50 steps
#     for t in range(50):
#         take (SetReverseAction(False), SetThrottleAction(1), SetBrakeAction(0))
#         wait

#     # Come to a stop for 20 steps
#     for t in range(20):
#         take (SetThrottleAction(0), SetBrakeAction(1))
#         wait

#     # Drive in reverse for 50 steps
#     for t in range(50):
#         take (SetReverseAction(True), SetThrottleAction(1), SetBrakeAction(0))
#         wait

# ego = new Car at (0, 0), with behavior ForwardThenStopThenReverse()

# behavior Forward():
#     while True:
#         take SetThrottleAction(1)

# behavior Stop():
#     while True:
#         take SetBrakeAction(1)

# behavior Reverse():
#     take SetReverseAction(True)
#     take SetBrakeAction(0)
#     while True:
#         take SetThrottleAction(1)

# behavior ForwardThenStopThenReverse():
#     do Forward() for 30 steps
#     do Stop() for 10 steps
#     do Reverse() for 10 steps

# ego = new Car at (0, 0), with behavior ForwardThenStopThenReverse
# ego = new Car with FollowLaneBehavior()

# --------
# behavior reverseThenGo():
#     take SetReverseAction(True)
#     for x in range(10):
#         take SetThrottleAction(1)
#     for x in range(30):
#         take SetBrakeAction(1), SetThrottleAction(0)
#     take SetReverseAction(False)
#     for x in range(10):
#         take SetThrottleAction(1), SetBrakeAction(0)

# ego = new Car with behavior reverseThenGo

# --------------------- works fine
# behavior DriveForAWhile():
#         do FollowLaneBehavior() for 30 seconds
#         take SetThrottleAction(0), SetBrakeAction(1)

# ego = new Car with behavior DriveForAWhile


#------------- car just goes forward one step???
# ego = new Car with velocity(1, 1)

#---------------------------------------- works
# behavior ReverseSlowly():
#     take SetReverseAction(True), SetThrottleAction(1), SetBrakeAction(0.5)

# ego = new Car with behavior ReverseSlowly

# -------------------------------------- test handbrake, works
# behavior GoAfterReleaseHandbrake():
#     for x in range(60):
#         take SetHandBrakeAction(True), SetThrottleAction(1)
#     take SetHandBrakeAction(False)

# ego = new Car with behavior GoAfterReleaseHandbrake
