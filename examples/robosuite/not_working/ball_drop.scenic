"""Ball drop test with recorded data for plotting"""

model scenic.simulators.robosuite.model

class Ball(Object):
    width: 0.2
    length: 0.2
    height: 0.2
    color: (1, 0, 0)

# Drop ball from 2 meters
ball = new Ball at (0, 0, 2.0)

# Record data for plotting
record ball.position[2] as height
record ball.velocity[2] as vertical_velocity  
record ball.speed as speed
record (2.0 - ball.position[2]) as distance_fallen

# Simple termination when ball settles
terminate when ball.position[2] < 0.15 and ball.speed < 0.01
terminate after 100 steps