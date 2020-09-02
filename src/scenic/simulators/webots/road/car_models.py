"""Car models built into Webots."""

import collections

CarModel = collections.namedtuple('CarModel', ('name', 'width', 'length'))
carModels = {
	CarModel('BmwX5', 2, 4.75),
	CarModel('CitroenCZero', 1.5, 3.5),
	CarModel('LincolnMKZ', 1.9, 5),
	CarModel('RangeRoverSportSVR', 1.9, 4.5),
	CarModel('ToyotaPrius', 1.9, 4.5),
	CarModel('Bus', 3, 10),
	CarModel('Truck', 2.7, 14.5),
	CarModel('Tractor', 2, 3),
	CarModel('MotorBikeSimple', 1, 2.5)
}
modelWithName = { model.name: model for model in carModels }

smallVehicles = { modelWithName[name] for name in {
	'BmwX5', 'CitroenCZero', 'LincolnMKZ', 'RangeRoverSportSVR', 'ToyotaPrius'
}}
