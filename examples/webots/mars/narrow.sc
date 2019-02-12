
from scenic.simulators.webots.mars.mars_model import *

ego = Robot at 0 @ -2

# Bottleneck made of two pipes with a rock in between

gap = 1.2 * ego.width
halfGap = gap / 2

bottleneck = OrientedPoint offset by (-1.5, 1.5) @ (0.5, 1.5), facing (-30, 30) deg

BigRock at bottleneck

leftEdge = OrientedPoint at bottleneck offset by -halfGap @ 0, facing (60, 120) deg relative to bottleneck.heading
rightEdge = OrientedPoint at bottleneck offset by halfGap @ 0, facing (-120, -60) deg relative to bottleneck.heading

Pipe ahead of leftEdge, with height (1, 2)
Pipe ahead of rightEdge, with height (1, 2)

# Other junk because why not?

Pipe
BigRock
BigRock
Rock
Rock
Rock