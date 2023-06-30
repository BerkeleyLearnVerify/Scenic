"""Car models built into Webots."""

from dataclasses import dataclass


@dataclass(frozen=True)
class CarModel:
    name: str
    width: float
    length: float
    height: float


carModels = {
    CarModel("BmwX5", 2, 4.75, 0),
    CarModel("CitroenCZero", 1.5, 3.5, 0),
    CarModel("LincolnMKZ", 1.9, 5, 0),
    CarModel("RangeRoverSportSVR", 1.9, 4.5, 0),
    CarModel("ToyotaPrius", 1.9, 4.5, 0),
    CarModel("Bus", 3, 10, 0),
    CarModel("Truck", 2.7, 14.5, 0),
    CarModel("Tractor", 2, 3, 0),
    CarModel("MotorBikeSimple", 1, 2.5, 0),
}
modelWithName = {model.name: model for model in carModels}

smallVehicles = {
    modelWithName[name]
    for name in {
        "BmwX5",
        "CitroenCZero",
        "LincolnMKZ",
        "RangeRoverSportSVR",
        "ToyotaPrius",
    }
}
