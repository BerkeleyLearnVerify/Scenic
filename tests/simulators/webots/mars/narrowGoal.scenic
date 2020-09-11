
from scenic.simulators.webots.mars.model import *

ego = Rover at 0 @ -2

goal = Goal at Range(-2, 2) @ Range(2, 2.5)

# Bottleneck made of two pipes with a rock in between

gap = 1.2 * ego.width
halfGap = gap / 2

bottleneck = OrientedPoint offset by Range(-1.5, 1.5) @ Range(0.5, 1.5), facing Range(-30, 30) deg

require abs((angle to goal) - (angle to bottleneck)) <= 10 deg

BigRock at bottleneck

leftEdge = OrientedPoint at bottleneck offset by -halfGap @ 0,
	facing Range(60, 120) deg relative to bottleneck.heading
rightEdge = OrientedPoint at bottleneck offset by halfGap @ 0,
	facing Range(-120, -60) deg relative to bottleneck.heading

Pipe ahead of leftEdge, with length Range(1, 2)
Pipe ahead of rightEdge, with length Range(1, 2)

# Other junk because why not?

Pipe
BigRock beyond bottleneck by Range(-0.5, 0.5) @ Range(0.5, 1)
BigRock beyond bottleneck by Range(-0.5, 0.5) @ Range(0.5, 1)
Rock
Rock
Rock