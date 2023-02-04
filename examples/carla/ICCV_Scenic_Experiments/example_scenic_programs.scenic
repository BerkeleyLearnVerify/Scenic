## Scenic program in Figure 3

ego = Car on road
otherCar = Car ahead of ego by Range(4,10)
require not (otherCar in intersection)



## Scenic program in Figure 6

spot = OrientedPoint on curb
ego = Car at (spot offset by (Range(2,4), Range(5,10)))
sideCar = Car left of spot by Range(1,3)