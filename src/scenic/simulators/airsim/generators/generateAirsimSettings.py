import argparse
import json

DEFAULT_DRONE_SETTINGS = {
    "VehicleType": "SimpleFlight",
    "AutoCreate": False,
    "Sensors": {
        "frontDistance": {"SensorType": 5, "Enabled": True, "DrawDebugPoints": True},
        "rightDistance": {
            "SensorType": 5,
            "Enabled": True,
            "DrawDebugPoints": True,
            "Yaw": 90,
            "Pitch": 0,
            "Roll": 0,
        },
        "leftDistance": {
            "SensorType": 5,
            "Enabled": True,
            "DrawDebugPoints": True,
            "Yaw": -90,
            "Pitch": 0,
            "Roll": 0,
        },
        "rearDistance": {
            "SensorType": 5,
            "Enabled": True,
            "DrawDebugPoints": True,
            "Yaw": 180,
            "Pitch": 0,
            "Roll": 0,
        },
    },
}
DEFAULT_MAX_DRONES = 10


parser = argparse.ArgumentParser()
parser.add_argument(
    "-o",
    "--outfile",
    type=str,
    help="The json file that the settings are written to.",
    required=True,
)
parser.add_argument(
    "-m",
    "--maxdrones",
    type=int,
    help="The maximum amount of drones your scenic program will support.",
    default=DEFAULT_MAX_DRONES,
)
parser.add_argument(
    "-dc",
    "--droneconfigpath",
    type=argparse.FileType("r"),
    help="The JSON containing only the config for 1 drone that will apply to all drones created. (not required)",
)

args = parser.parse_args()

if not args.outfile.endswith("json"):
    raise argparse.ArgumentTypeError(
            "The --outfile argument must be a json file."
        )

droneConfig = None
if args.droneconfigpath:
    if not args.droneconfigpath.name.endswith("json"):
        raise argparse.ArgumentTypeError(
            "The --droneconfigpath argument must be a json file."
        )

    droneConfig = json.load(args.droneconfigpath)
else:
    droneConfig = DEFAULT_DRONE_SETTINGS

maxDrones = args.maxdrones

settings = {"SimMode": "Multirotor", "ClockSpeed": 1, "Vehicles": {}}


for i in range(maxDrones):
    settings["Vehicles"]["Drone" + str(i)] = droneConfig.copy()
    drone = settings["Vehicles"]["Drone" + str(i)]
    if i == 0:
        drone["AutoCreate"] = True


with open(args.outfile, "w") as outfile:
    json.dump(settings, outfile, indent=4)

print("created settings at", args.outfile)
