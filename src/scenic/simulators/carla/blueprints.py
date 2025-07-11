"""
CARLA blueprints for cars, pedestrians, etc.

Defines CARLA Actor blueprints for Scenic, with version-specific overrides.

At import time we detect the installed CARLA Python package version and
select the appropriate sets of blueprint IDs for vehicles, pedestrians,
props, etc.  If a blueprint is renamed in a newer CARLA, we map old names
via `oldBlueprintNames`.

NOTE: CARLA 0.10.x currently ships *no* bicycle, motorcycle, box or
iron-plate props—those appear only in ≤0.9.x.
"""

from importlib.metadata import PackageNotFoundError, version

# ---------------------------------------------------------------------------
# Detect CARLA version
# ---------------------------------------------------------------------------

try:
    carla_pkg_version = version("carla")
except PackageNotFoundError:
    # during docs builds or no client installed, assume an older version
    carla_pkg_version = "0.0.0"

is_carla_0_10 = carla_pkg_version.startswith("0.10")

# ---------------------------------------------------------------------------
# Legacy name mappings
# ---------------------------------------------------------------------------

#: Mapping from current names of blueprints to ones in old CARLA versions.
#: We provide a tuple of old names in case they change more than once.
oldBlueprintNames = {
    "vehicle.dodge.charger_police": ("vehicle.dodge_charger.police",),
    "vehicle.lincoln.mkz_2017": ("vehicle.lincoln.mkz2017",),
    "vehicle.mercedes.coupe": ("vehicle.mercedes-benz.coupe",),
    "vehicle.mini.cooper_s": ("vehicle.mini.cooperst",),
    "vehicle.ford.mustang": ("vehicle.mustang.mustang",),
}

# ---------------------------------------------------------------------------
# Version-specific blueprint lists
# ---------------------------------------------------------------------------

if is_carla_0_10:
    # -- CARLA 0.10.x -------------------------------------------------------

    #: blueprints for cars in CARLA 0.10.x
    carModels = [
        "vehicle.taxi.ford",
        "vehicle.dodgecop.charger",
        "vehicle.dodge.charger",
        "vehicle.nissan.patrol",
        "vehicle.mini.cooper",
        "vehicle.lincoln.mkz",
    ]

    #: blueprints for trucks in CARLA 0.10.x
    truckModels = [
        "vehicle.ambulance.ford",
        "vehicle.sprinter.mercedes",
        "vehicle.fuso.mitsubishi",
        "vehicle.carlacola.actors",
        "vehicle.firetruck.actors",
    ]

    #: blueprints for trash cans in CARLA 0.10.x
    trashModels = [
        "static.prop.trashcan01",
        "static.prop.trashcan02",
        "static.prop.trashcan03",
        "static.prop.trashcan04",
    ]

    #: blueprints for containers in CARLA 0.10.x
    containerModels = [
        "static.prop.container",
    ]

    #: blueprints for tables in CARLA 0.10.x
    tableModels = [
        "static.prop.plastictable",
    ]

    #: blueprints for flowerpots in CARLA 0.10.x
    plantpotModels = [
        "static.prop.plantpot01",
        "static.prop.plantpot02",
        "static.prop.plantpot03",
        "static.prop.plantpot04",
        "static.prop.plantpot05",
        "static.prop.plantpot06",
        "static.prop.plantpot07",
    ]

    #: blueprints for creased boxes in CARLA 0.10.x
    creasedboxModels = [
        "static.prop.creasedbox01",
    ]

    #: blueprints for benches in CARLA 0.10.x
    benchModels = [
        "static.prop.bench01",
        "static.prop.bench02",
    ]

    #: blueprints for pedestrians in CARLA 0.10.x
    walkerModels = [
        "walker.pedestrian.0015",
        "walker.pedestrian.0016",
        "walker.pedestrian.0017",
        "walker.pedestrian.0018",
        "walker.pedestrian.0019",
        "walker.pedestrian.0020",
        "walker.pedestrian.0021",
        "walker.pedestrian.0022",
        "walker.pedestrian.0023",
        "walker.pedestrian.0024",
        "walker.pedestrian.0025",
        "walker.pedestrian.0026",
        "walker.pedestrian.0027",
        "walker.pedestrian.0028",
    ]

    #: blueprints for bicycles (none in 0.10.x)
    bicycleModels = []

    #: blueprints for motorcycles (none in 0.10.x)
    motorcycleModels = []

    #: blueprints for boxes (none in 0.10.x)
    boxModels = []

    #: blueprints for iron plates (none in 0.10.x)
    ironplateModels = []

else:
    # -- CARLA ≤0.9.x -------------------------------------------------------

    #: blueprints for cars
    carModels = [
        "vehicle.audi.a2",
        "vehicle.audi.etron",
        "vehicle.audi.tt",
        "vehicle.bmw.grandtourer",
        "vehicle.chevrolet.impala",
        "vehicle.citroen.c3",
        "vehicle.dodge.charger_police",
        "vehicle.jeep.wrangler_rubicon",
        "vehicle.lincoln.mkz_2017",
        "vehicle.mercedes.coupe",
        "vehicle.mini.cooper_s",
        "vehicle.ford.mustang",
        "vehicle.nissan.micra",
        "vehicle.nissan.patrol",
        "vehicle.seat.leon",
        "vehicle.tesla.model3",
        "vehicle.toyota.prius",
        "vehicle.volkswagen.t2",
    ]

    #: blueprints for trucks
    truckModels = [
        "vehicle.carlamotors.carlacola",
        "vehicle.tesla.cybertruck",
    ]

    #: blueprints for trash cans
    trashModels = [
        "static.prop.trashcan01",
        "static.prop.trashcan02",
        "static.prop.trashcan03",
        "static.prop.trashcan04",
        "static.prop.trashcan05",
        "static.prop.bin",
    ]

    #: blueprints for containers
    containerModels = [
        "static.prop.container",
        "static.prop.clothcontainer",
        "static.prop.glasscontainer",
    ]

    #: blueprints for tables
    tableModels = [
        "static.prop.table",
        "static.prop.plastictable",
    ]

    #: blueprints for flowerpots
    plantpotModels = [
        "static.prop.plantpot01",
        "static.prop.plantpot02",
        "static.prop.plantpot03",
        "static.prop.plantpot04",
        "static.prop.plantpot05",
        "static.prop.plantpot06",
        "static.prop.plantpot07",
        "static.prop.plantpot08",
    ]

    #: blueprints for creased boxes
    creasedboxModels = [
        "static.prop.creasedbox01",
        "static.prop.creasedbox02",
        "static.prop.creasedbox03",
    ]

    #: blueprints for benches
    benchModels = [
        "static.prop.bench01",
        "static.prop.bench02",
        "static.prop.bench03",
    ]

    #: blueprints for pedestrians
    walkerModels = [
        "walker.pedestrian.0001",
        "walker.pedestrian.0002",
        "walker.pedestrian.0003",
        "walker.pedestrian.0004",
        "walker.pedestrian.0005",
        "walker.pedestrian.0006",
        "walker.pedestrian.0007",
        "walker.pedestrian.0008",
        "walker.pedestrian.0009",
        "walker.pedestrian.0010",
        "walker.pedestrian.0011",
        "walker.pedestrian.0012",
        "walker.pedestrian.0013",
        "walker.pedestrian.0014",
    ]

    #: blueprints for bicycles
    bicycleModels = [
        "vehicle.bh.crossbike",
        "vehicle.diamondback.century",
        "vehicle.gazelle.omafiets",
    ]

    #: blueprints for motorcycles
    motorcycleModels = [
        "vehicle.harley-davidson.low_rider",
        "vehicle.kawasaki.ninja",
        "vehicle.yamaha.yzf",
    ]

    #: blueprints for boxes
    boxModels = [
        "static.prop.box01",
        "static.prop.box02",
        "static.prop.box03",
    ]

    #: blueprints for iron plates
    ironplateModels = [
        "static.prop.ironplank",
    ]

# ---------------------------------------------------------------------------
# Models shared by all versions
# ---------------------------------------------------------------------------

#: blueprints for traffic cones
coneModels = [
    "static.prop.constructioncone",
    "static.prop.trafficcone01",
    "static.prop.trafficcone02",
]

#: blueprints for road debris
debrisModels = [
    "static.prop.dirtdebris01",
    "static.prop.dirtdebris02",
    "static.prop.dirtdebris03",
]

#: blueprints for vending machines
vendingMachineModels = [
    "static.prop.vendingmachine",
]

#: blueprints for chairs
chairModels = [
    "static.prop.plasticchair",
]

#: blueprints for bus stops
busStopModels = [
    "static.prop.busstop",
]

#: blueprints for roadside billboards
advertisementModels = [
    "static.prop.advertisement",
    "static.prop.streetsign",
    "static.prop.streetsign01",
    "static.prop.streetsign04",
]

#: blueprints for pieces of trash
garbageModels = [
    "static.prop.colacan",
    "static.prop.garbage01",
    "static.prop.garbage02",
    "static.prop.garbage03",
    "static.prop.garbage04",
    "static.prop.garbage05",
    "static.prop.garbage06",
    "static.prop.plasticbag",
    "static.prop.trashbag",
]

#: blueprints for traffic barriers
barrierModels = [
    "static.prop.streetbarrier",
    "static.prop.chainbarrier",
    "static.prop.chainbarrierend",
]

#: blueprints for mailboxes
mailboxModels = [
    "static.prop.mailbox",
]

#: blueprints for garden gnomes
gnomeModels = [
    "static.prop.gnome",
]

#: blueprints for briefcases, suitcases, etc.
caseModels = [
    "static.prop.travelcase",
    "static.prop.briefcase",
    "static.prop.guitarcase",
]

#: blueprints for barrels
barrelModels = [
    "static.prop.barrel",
]

#: blueprints for ATMs
atmModels = [
    "static.prop.atm",
]

#: blueprints for kiosks
kioskModels = [
    "static.prop.kiosk_01",
]

#: blueprints for traffic warning signs
trafficwarningModels = [
    "static.prop.trafficwarning",
]
