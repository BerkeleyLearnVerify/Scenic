""" Scenario Description:
There are two cars on an intersection, and the cars are heading an opposite direction
such that their heading angle difference is greater than 140 degrees. 
And, other cars' headings differs from the ego's by 50 to 135 degrees to the left.
The other2's heading differs from the ego's by 50 to 135 degrees to the right.
"""

model scenic.simulators.carla.model

ego = Car on drivableRoad, 
		facing Range(-15,15) deg relative to roadDirection,
		with visibleDistance 50,
		with viewAngle 135 deg
other1 = Car on intersection,
			facing Range(50,135) deg relative to ego.heading
other2 = Car on intersection,
			facing -1 * Range(50,135) deg relative to ego.heading

require abs(relative heading of other1 from other2) > 100 deg
require (distance from ego to intersectionRegion) < 10