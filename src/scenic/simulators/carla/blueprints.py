# AUTO-GENERATED. Do not edit by hand.
# Built from tools/carla/make_blueprints.py

"""CARLA blueprints for cars, pedestrians, etc."""

from importlib.metadata import PackageNotFoundError, version as _pkg_version
import warnings

from scenic.core.distributions import distributionFunction
from scenic.core.errors import InvalidScenarioError

try:
    _CARLA_VER = _pkg_version("carla")
except PackageNotFoundError:
    _CARLA_VER = "0.0.0"  # no carla package; default to newest known blueprints


def _verkey(s: str):
    # Handle '0.9.15', '0.10.0', '1.2' -> (major, minor, patch)
    parts = [int(p) for p in s.split(".")]
    parts += [0] * (3 - len(parts))
    return tuple(parts[:3])


def _pick(vermap, ver):
    """Choose the blueprint map for CARLA version"""
    # 1) Exact match
    if ver in vermap:
        return vermap[ver]
    # 2) If same major.minor, use the newest patch.
    mm = ".".join(ver.split(".")[:2])
    cands = [v for v in vermap if v.startswith(mm + ".")]
    if cands:
        best = max(cands, key=_verkey)
        return vermap[best]
    # 3) Otherwise, use the newest version.
    best = max(vermap.keys(), key=_verkey)
    if ver != "0.0.0":
        warnings.warn(
            f"Unknown CARLA version {ver}; using blueprints for {best}."
            "Scenic may not have the correct set of blueprints."
        )
    return vermap[best]


oldBlueprintNames = {
    # Map current names to legacy names
    "vehicle.dodge.charger_police": ("vehicle.dodge_charger.police",),
    "vehicle.lincoln.mkz_2017": ("vehicle.lincoln.mkz2017",),
    "vehicle.mercedes.coupe": ("vehicle.mercedes-benz.coupe",),
    "vehicle.mini.cooper_s": ("vehicle.mini.cooperst",),
    "vehicle.ford.mustang": ("vehicle.mustang.mustang",),
}

_IDS = {
    "0.10.0": {
        "advertisementModels": [
            "static.prop.advertisement",
            "static.prop.streetsign",
            "static.prop.streetsign01",
            "static.prop.streetsign04",
        ],
        "atmModels": ["static.prop.atm"],
        "barrelModels": ["static.prop.barrel"],
        "barrierModels": [
            "static.prop.chainbarrier",
            "static.prop.chainbarrierend",
            "static.prop.streetbarrier",
        ],
        "benchModels": ["static.prop.bench01", "static.prop.bench02"],
        "bicycleModels": [],
        "boxModels": [],
        "busModels": ["vehicle.fuso.mitsubishi"],
        "busStopModels": ["static.prop.busstop"],
        "carModels": [
            "vehicle.dodge.charger",
            "vehicle.dodgecop.charger",
            "vehicle.lincoln.mkz",
            "vehicle.mini.cooper",
            "vehicle.nissan.patrol",
            "vehicle.taxi.ford",
        ],
        "caseModels": [
            "static.prop.briefcase",
            "static.prop.guitarcase",
            "static.prop.travelcase",
        ],
        "chairModels": ["static.prop.plasticchair"],
        "coneModels": [
            "static.prop.constructioncone",
            "static.prop.trafficcone01",
            "static.prop.trafficcone02",
        ],
        "containerModels": ["static.prop.container"],
        "creasedboxModels": ["static.prop.creasedbox01"],
        "debrisModels": [
            "static.prop.dirtdebris01",
            "static.prop.dirtdebris02",
            "static.prop.dirtdebris03",
        ],
        "garbageModels": [
            "static.prop.colacan",
            "static.prop.garbage01",
            "static.prop.garbage02",
            "static.prop.garbage03",
            "static.prop.garbage04",
            "static.prop.garbage05",
            "static.prop.garbage06",
            "static.prop.plasticbag",
            "static.prop.platformgarbage01",
            "static.prop.trashbag",
        ],
        "gnomeModels": ["static.prop.gnome"],
        "ironplateModels": [],
        "kioskModels": ["static.prop.kiosk_01"],
        "mailboxModels": ["static.prop.mailbox"],
        "motorcycleModels": [],
        "plantpotModels": [
            "static.prop.plantpot01",
            "static.prop.plantpot02",
            "static.prop.plantpot03",
            "static.prop.plantpot04",
            "static.prop.plantpot05",
            "static.prop.plantpot06",
            "static.prop.plantpot07",
        ],
        "tableModels": ["static.prop.maptable", "static.prop.plastictable"],
        "trafficwarningModels": ["static.prop.trafficwarning"],
        "trashModels": [
            "static.prop.trashcan01",
            "static.prop.trashcan02",
            "static.prop.trashcan03",
            "static.prop.trashcan04",
        ],
        "truckModels": [
            "vehicle.ambulance.ford",
            "vehicle.carlacola.actors",
            "vehicle.firetruck.actors",
        ],
        "vanModels": ["vehicle.sprinter.mercedes"],
        "vendingMachineModels": ["static.prop.vendingmachine"],
        "walkerModels": [
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
            "walker.pedestrian.0029",
            "walker.pedestrian.0030",
            "walker.pedestrian.0031",
            "walker.pedestrian.0032",
            "walker.pedestrian.0033",
            "walker.pedestrian.0034",
            "walker.pedestrian.0035",
            "walker.pedestrian.0036",
            "walker.pedestrian.0037",
            "walker.pedestrian.0038",
            "walker.pedestrian.0039",
            "walker.pedestrian.0040",
            "walker.pedestrian.0041",
            "walker.pedestrian.0042",
            "walker.pedestrian.0043",
            "walker.pedestrian.0044",
            "walker.pedestrian.0045",
            "walker.pedestrian.0046",
            "walker.pedestrian.0047",
            "walker.pedestrian.0048",
            "walker.pedestrian.0049",
            "walker.pedestrian.0050",
            "walker.pedestrian.0051",
        ],
    },
    "0.9.16": {
        "advertisementModels": [
            "static.prop.advertisement",
            "static.prop.streetsign",
            "static.prop.streetsign01",
            "static.prop.streetsign04",
        ],
        "atmModels": ["static.prop.atm"],
        "barrelModels": ["static.prop.barrel"],
        "barrierModels": [
            "static.prop.chainbarrier",
            "static.prop.chainbarrierend",
            "static.prop.streetbarrier",
        ],
        "benchModels": [
            "static.prop.bench01",
            "static.prop.bench02",
            "static.prop.bench03",
        ],
        "bicycleModels": [
            "vehicle.bh.crossbike",
            "vehicle.diamondback.century",
            "vehicle.gazelle.omafiets",
        ],
        "boxModels": ["static.prop.box01", "static.prop.box02", "static.prop.box03"],
        "busModels": ["vehicle.mitsubishi.fusorosa"],
        "busStopModels": ["static.prop.busstop", "static.prop.busstoplb"],
        "carModels": [
            "vehicle.audi.a2",
            "vehicle.audi.etron",
            "vehicle.audi.tt",
            "vehicle.bmw.grandtourer",
            "vehicle.chevrolet.impala",
            "vehicle.citroen.c3",
            "vehicle.dodge.charger_2020",
            "vehicle.dodge.charger_police",
            "vehicle.dodge.charger_police_2020",
            "vehicle.ford.crown",
            "vehicle.ford.mustang",
            "vehicle.jeep.wrangler_rubicon",
            "vehicle.lincoln.mkz_2017",
            "vehicle.lincoln.mkz_2020",
            "vehicle.mercedes.coupe",
            "vehicle.mercedes.coupe_2020",
            "vehicle.micro.microlino",
            "vehicle.mini.cooper_s",
            "vehicle.mini.cooper_s_2021",
            "vehicle.nissan.micra",
            "vehicle.nissan.patrol",
            "vehicle.nissan.patrol_2021",
            "vehicle.seat.leon",
            "vehicle.tesla.model3",
            "vehicle.toyota.prius",
        ],
        "caseModels": [
            "static.prop.briefcase",
            "static.prop.guitarcase",
            "static.prop.travelcase",
        ],
        "chairModels": ["static.prop.plasticchair"],
        "coneModels": [
            "static.prop.constructioncone",
            "static.prop.trafficcone01",
            "static.prop.trafficcone02",
        ],
        "containerModels": [
            "static.prop.clothcontainer",
            "static.prop.container",
            "static.prop.glasscontainer",
        ],
        "creasedboxModels": [
            "static.prop.creasedbox01",
            "static.prop.creasedbox02",
            "static.prop.creasedbox03",
        ],
        "debrisModels": [
            "static.prop.dirtdebris01",
            "static.prop.dirtdebris02",
            "static.prop.dirtdebris03",
        ],
        "garbageModels": [
            "static.prop.colacan",
            "static.prop.garbage01",
            "static.prop.garbage02",
            "static.prop.garbage03",
            "static.prop.garbage04",
            "static.prop.garbage05",
            "static.prop.garbage06",
            "static.prop.plasticbag",
            "static.prop.platformgarbage01",
            "static.prop.trashbag",
        ],
        "gnomeModels": ["static.prop.gnome"],
        "ironplateModels": ["static.prop.ironplank"],
        "kioskModels": ["static.prop.kiosk_01"],
        "mailboxModels": ["static.prop.mailbox"],
        "motorcycleModels": [
            "vehicle.harley-davidson.low_rider",
            "vehicle.kawasaki.ninja",
            "vehicle.vespa.zx125",
            "vehicle.yamaha.yzf",
        ],
        "plantpotModels": [
            "static.prop.plantpot01",
            "static.prop.plantpot02",
            "static.prop.plantpot03",
            "static.prop.plantpot04",
            "static.prop.plantpot05",
            "static.prop.plantpot06",
            "static.prop.plantpot07",
            "static.prop.plantpot08",
        ],
        "tableModels": [
            "static.prop.maptable",
            "static.prop.plastictable",
            "static.prop.table",
        ],
        "trafficwarningModels": ["static.prop.trafficwarning"],
        "trashModels": [
            "static.prop.bin",
            "static.prop.trashcan01",
            "static.prop.trashcan02",
            "static.prop.trashcan03",
            "static.prop.trashcan04",
            "static.prop.trashcan05",
        ],
        "truckModels": [
            "vehicle.carlamotors.carlacola",
            "vehicle.carlamotors.european_hgv",
            "vehicle.carlamotors.firetruck",
            "vehicle.tesla.cybertruck",
        ],
        "vanModels": [
            "vehicle.ford.ambulance",
            "vehicle.mercedes.sprinter",
            "vehicle.volkswagen.t2",
            "vehicle.volkswagen.t2_2021",
        ],
        "vendingMachineModels": ["static.prop.vendingmachine"],
        "walkerModels": [
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
            "walker.pedestrian.0029",
            "walker.pedestrian.0030",
            "walker.pedestrian.0031",
            "walker.pedestrian.0032",
            "walker.pedestrian.0033",
            "walker.pedestrian.0034",
            "walker.pedestrian.0035",
            "walker.pedestrian.0036",
            "walker.pedestrian.0037",
            "walker.pedestrian.0038",
            "walker.pedestrian.0039",
            "walker.pedestrian.0040",
            "walker.pedestrian.0041",
            "walker.pedestrian.0042",
            "walker.pedestrian.0043",
            "walker.pedestrian.0044",
            "walker.pedestrian.0045",
            "walker.pedestrian.0046",
            "walker.pedestrian.0047",
            "walker.pedestrian.0048",
            "walker.pedestrian.0049",
            "walker.pedestrian.0050",
            "walker.pedestrian.0051",
            "walker.pedestrian.0052",
        ],
    },
    "0.9.15": {
        "advertisementModels": [
            "static.prop.advertisement",
            "static.prop.streetsign",
            "static.prop.streetsign01",
            "static.prop.streetsign04",
        ],
        "atmModels": ["static.prop.atm"],
        "barrelModels": ["static.prop.barrel"],
        "barrierModels": [
            "static.prop.chainbarrier",
            "static.prop.chainbarrierend",
            "static.prop.streetbarrier",
        ],
        "benchModels": [
            "static.prop.bench01",
            "static.prop.bench02",
            "static.prop.bench03",
        ],
        "bicycleModels": [
            "vehicle.bh.crossbike",
            "vehicle.diamondback.century",
            "vehicle.gazelle.omafiets",
        ],
        "boxModels": ["static.prop.box01", "static.prop.box02", "static.prop.box03"],
        "busModels": ["vehicle.mitsubishi.fusorosa"],
        "busStopModels": ["static.prop.busstop", "static.prop.busstoplb"],
        "carModels": [
            "vehicle.audi.a2",
            "vehicle.audi.etron",
            "vehicle.audi.tt",
            "vehicle.bmw.grandtourer",
            "vehicle.chevrolet.impala",
            "vehicle.citroen.c3",
            "vehicle.dodge.charger_2020",
            "vehicle.dodge.charger_police",
            "vehicle.dodge.charger_police_2020",
            "vehicle.ford.crown",
            "vehicle.ford.mustang",
            "vehicle.jeep.wrangler_rubicon",
            "vehicle.lincoln.mkz_2017",
            "vehicle.lincoln.mkz_2020",
            "vehicle.mercedes.coupe",
            "vehicle.mercedes.coupe_2020",
            "vehicle.micro.microlino",
            "vehicle.mini.cooper_s",
            "vehicle.mini.cooper_s_2021",
            "vehicle.nissan.micra",
            "vehicle.nissan.patrol",
            "vehicle.nissan.patrol_2021",
            "vehicle.seat.leon",
            "vehicle.tesla.model3",
            "vehicle.toyota.prius",
        ],
        "caseModels": [
            "static.prop.briefcase",
            "static.prop.guitarcase",
            "static.prop.travelcase",
        ],
        "chairModels": ["static.prop.plasticchair"],
        "coneModels": [
            "static.prop.constructioncone",
            "static.prop.trafficcone01",
            "static.prop.trafficcone02",
        ],
        "containerModels": [
            "static.prop.clothcontainer",
            "static.prop.container",
            "static.prop.glasscontainer",
        ],
        "creasedboxModels": [
            "static.prop.creasedbox01",
            "static.prop.creasedbox02",
            "static.prop.creasedbox03",
        ],
        "debrisModels": [
            "static.prop.dirtdebris01",
            "static.prop.dirtdebris02",
            "static.prop.dirtdebris03",
        ],
        "garbageModels": [
            "static.prop.colacan",
            "static.prop.garbage01",
            "static.prop.garbage02",
            "static.prop.garbage03",
            "static.prop.garbage04",
            "static.prop.garbage05",
            "static.prop.garbage06",
            "static.prop.plasticbag",
            "static.prop.platformgarbage01",
            "static.prop.trashbag",
        ],
        "gnomeModels": ["static.prop.gnome"],
        "ironplateModels": ["static.prop.ironplank"],
        "kioskModels": ["static.prop.kiosk_01"],
        "mailboxModels": ["static.prop.mailbox"],
        "motorcycleModels": [
            "vehicle.harley-davidson.low_rider",
            "vehicle.kawasaki.ninja",
            "vehicle.vespa.zx125",
            "vehicle.yamaha.yzf",
        ],
        "plantpotModels": [
            "static.prop.plantpot01",
            "static.prop.plantpot02",
            "static.prop.plantpot03",
            "static.prop.plantpot04",
            "static.prop.plantpot05",
            "static.prop.plantpot06",
            "static.prop.plantpot07",
            "static.prop.plantpot08",
        ],
        "tableModels": [
            "static.prop.maptable",
            "static.prop.plastictable",
            "static.prop.table",
        ],
        "trafficwarningModels": ["static.prop.trafficwarning"],
        "trashModels": [
            "static.prop.bin",
            "static.prop.trashcan01",
            "static.prop.trashcan02",
            "static.prop.trashcan03",
            "static.prop.trashcan04",
            "static.prop.trashcan05",
        ],
        "truckModels": [
            "vehicle.carlamotors.carlacola",
            "vehicle.carlamotors.european_hgv",
            "vehicle.carlamotors.firetruck",
            "vehicle.tesla.cybertruck",
        ],
        "vanModels": [
            "vehicle.ford.ambulance",
            "vehicle.mercedes.sprinter",
            "vehicle.volkswagen.t2",
            "vehicle.volkswagen.t2_2021",
        ],
        "vendingMachineModels": ["static.prop.vendingmachine"],
        "walkerModels": [
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
            "walker.pedestrian.0029",
            "walker.pedestrian.0030",
            "walker.pedestrian.0031",
            "walker.pedestrian.0032",
            "walker.pedestrian.0033",
            "walker.pedestrian.0034",
            "walker.pedestrian.0035",
            "walker.pedestrian.0036",
            "walker.pedestrian.0037",
            "walker.pedestrian.0038",
            "walker.pedestrian.0039",
            "walker.pedestrian.0040",
            "walker.pedestrian.0041",
            "walker.pedestrian.0042",
            "walker.pedestrian.0043",
            "walker.pedestrian.0044",
            "walker.pedestrian.0045",
            "walker.pedestrian.0046",
            "walker.pedestrian.0047",
            "walker.pedestrian.0048",
            "walker.pedestrian.0049",
            "walker.pedestrian.0050",
            "walker.pedestrian.0051",
        ],
    },
    "0.9.14": {
        "advertisementModels": [
            "static.prop.advertisement",
            "static.prop.streetsign",
            "static.prop.streetsign01",
            "static.prop.streetsign04",
        ],
        "atmModels": ["static.prop.atm"],
        "barrelModels": ["static.prop.barrel"],
        "barrierModels": [
            "static.prop.chainbarrier",
            "static.prop.chainbarrierend",
            "static.prop.streetbarrier",
        ],
        "benchModels": [
            "static.prop.bench01",
            "static.prop.bench02",
            "static.prop.bench03",
        ],
        "bicycleModels": [
            "vehicle.bh.crossbike",
            "vehicle.diamondback.century",
            "vehicle.gazelle.omafiets",
        ],
        "boxModels": ["static.prop.box01", "static.prop.box02", "static.prop.box03"],
        "busModels": ["vehicle.mitsubishi.fusorosa"],
        "busStopModels": ["static.prop.busstop", "static.prop.busstoplb"],
        "carModels": [
            "vehicle.audi.a2",
            "vehicle.audi.etron",
            "vehicle.audi.tt",
            "vehicle.bmw.grandtourer",
            "vehicle.chevrolet.impala",
            "vehicle.citroen.c3",
            "vehicle.dodge.charger_2020",
            "vehicle.dodge.charger_police",
            "vehicle.dodge.charger_police_2020",
            "vehicle.ford.crown",
            "vehicle.ford.mustang",
            "vehicle.jeep.wrangler_rubicon",
            "vehicle.lincoln.mkz_2017",
            "vehicle.lincoln.mkz_2020",
            "vehicle.mercedes.coupe",
            "vehicle.mercedes.coupe_2020",
            "vehicle.micro.microlino",
            "vehicle.mini.cooper_s",
            "vehicle.mini.cooper_s_2021",
            "vehicle.nissan.micra",
            "vehicle.nissan.patrol",
            "vehicle.nissan.patrol_2021",
            "vehicle.seat.leon",
            "vehicle.tesla.model3",
            "vehicle.toyota.prius",
        ],
        "caseModels": [
            "static.prop.briefcase",
            "static.prop.guitarcase",
            "static.prop.travelcase",
        ],
        "chairModels": ["static.prop.plasticchair"],
        "coneModels": [
            "static.prop.constructioncone",
            "static.prop.trafficcone01",
            "static.prop.trafficcone02",
        ],
        "containerModels": [
            "static.prop.clothcontainer",
            "static.prop.container",
            "static.prop.glasscontainer",
        ],
        "creasedboxModels": [
            "static.prop.creasedbox01",
            "static.prop.creasedbox02",
            "static.prop.creasedbox03",
        ],
        "debrisModels": [
            "static.prop.dirtdebris01",
            "static.prop.dirtdebris02",
            "static.prop.dirtdebris03",
        ],
        "garbageModels": [
            "static.prop.colacan",
            "static.prop.garbage01",
            "static.prop.garbage02",
            "static.prop.garbage03",
            "static.prop.garbage04",
            "static.prop.garbage05",
            "static.prop.garbage06",
            "static.prop.plasticbag",
            "static.prop.platformgarbage01",
            "static.prop.trashbag",
        ],
        "gnomeModels": ["static.prop.gnome"],
        "ironplateModels": ["static.prop.ironplank"],
        "kioskModels": ["static.prop.kiosk_01"],
        "mailboxModels": ["static.prop.mailbox"],
        "motorcycleModels": [
            "vehicle.harley-davidson.low_rider",
            "vehicle.kawasaki.ninja",
            "vehicle.vespa.zx125",
            "vehicle.yamaha.yzf",
        ],
        "plantpotModels": [
            "static.prop.plantpot01",
            "static.prop.plantpot02",
            "static.prop.plantpot03",
            "static.prop.plantpot04",
            "static.prop.plantpot05",
            "static.prop.plantpot06",
            "static.prop.plantpot07",
            "static.prop.plantpot08",
        ],
        "tableModels": [
            "static.prop.maptable",
            "static.prop.plastictable",
            "static.prop.table",
        ],
        "trafficwarningModels": ["static.prop.trafficwarning"],
        "trashModels": [
            "static.prop.bin",
            "static.prop.trashcan01",
            "static.prop.trashcan02",
            "static.prop.trashcan03",
            "static.prop.trashcan04",
            "static.prop.trashcan05",
        ],
        "truckModels": [
            "vehicle.carlamotors.carlacola",
            "vehicle.carlamotors.firetruck",
            "vehicle.tesla.cybertruck",
        ],
        "vanModels": [
            "vehicle.ford.ambulance",
            "vehicle.mercedes.sprinter",
            "vehicle.volkswagen.t2",
            "vehicle.volkswagen.t2_2021",
        ],
        "vendingMachineModels": ["static.prop.vendingmachine"],
        "walkerModels": [
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
            "walker.pedestrian.0029",
            "walker.pedestrian.0030",
            "walker.pedestrian.0031",
            "walker.pedestrian.0032",
            "walker.pedestrian.0033",
            "walker.pedestrian.0034",
            "walker.pedestrian.0035",
            "walker.pedestrian.0036",
            "walker.pedestrian.0037",
            "walker.pedestrian.0038",
            "walker.pedestrian.0039",
            "walker.pedestrian.0040",
            "walker.pedestrian.0041",
            "walker.pedestrian.0042",
            "walker.pedestrian.0043",
            "walker.pedestrian.0044",
            "walker.pedestrian.0045",
            "walker.pedestrian.0046",
            "walker.pedestrian.0047",
            "walker.pedestrian.0048",
            "walker.pedestrian.0049",
        ],
    },
}

_DIMS = {
    "0.10.0": {
        "static.prop.advertisement": {
            "width": 0.28684958815574646,
            "length": 1.549233317375183,
            "height": 2.4428441524505615,
        },
        "static.prop.atm": {
            "width": 0.8325381278991699,
            "length": 0.5721203684806824,
            "height": 2.194814443588257,
        },
        "static.prop.barbeque": {
            "width": 0.49884626269340515,
            "length": 0.9456468820571899,
            "height": 1.2640891075134277,
        },
        "static.prop.barrel": {
            "width": 0.47143638134002686,
            "length": 0.4596165418624878,
            "height": 0.8067459464073181,
        },
        "static.prop.bench01": {
            "width": 0.6381909847259521,
            "length": 1.7933329343795776,
            "height": 1.0038363933563232,
        },
        "static.prop.bench02": {
            "width": 0.5126523971557617,
            "length": 1.58349609375,
            "height": 0.5126523971557617,
        },
        "static.prop.bike helmet": {
            "width": 0.2770470678806305,
            "length": 0.18554961681365967,
            "height": 0.1712976098060608,
        },
        "static.prop.briefcase": {
            "width": 0.18117640912532806,
            "length": 0.543353796005249,
            "height": 0.43285050988197327,
        },
        "static.prop.brokentile01": {
            "width": 0.1760428547859192,
            "length": 0.13444176316261292,
            "height": 0.006563859060406685,
        },
        "static.prop.brokentile02": {
            "width": 0.15229617059230804,
            "length": 0.20967701077461243,
            "height": 0.007594939786940813,
        },
        "static.prop.brokentile03": {
            "width": 0.1449233442544937,
            "length": 0.1544525921344757,
            "height": 0.008505801670253277,
        },
        "static.prop.brokentile04": {
            "width": 0.12647496163845062,
            "length": 0.12972931563854218,
            "height": 0.01014416478574276,
        },
        "static.prop.busstop": {
            "width": 3.8760786056518555,
            "length": 1.8936100006103516,
            "height": 2.738938331604004,
        },
        "static.prop.calibrator": {
            "width": 0.690200924873352,
            "length": 1.5475953817367554,
            "height": 0.20195052027702332,
        },
        "static.prop.chainbarrier": {
            "width": 0.24313338100910187,
            "length": 1.6833475828170776,
            "height": 0.6920976042747498,
        },
        "static.prop.chainbarrierend": {
            "width": 0.24313338100910187,
            "length": 0.24313347041606903,
            "height": 0.6920976042747498,
        },
        "static.prop.colacan": {
            "width": 0.07167824357748032,
            "length": 0.06817007064819336,
            "height": 0.10954885929822922,
        },
        "static.prop.constructioncone": {
            "width": 0.4784207046031952,
            "length": 0.478452205657959,
            "height": 0.7117974162101746,
        },
        "static.prop.container": {
            "width": 1.6529669761657715,
            "length": 5.399704933166504,
            "height": 1.3477081060409546,
        },
        "static.prop.creasedbox01": {
            "width": 0.9068000316619873,
            "length": 1.0287128686904907,
            "height": 0.061725616455078125,
        },
        "static.prop.dirtdebris01": {
            "width": 1.5139321088790894,
            "length": 1.8574368953704834,
            "height": 0.14900343120098114,
        },
        "static.prop.dirtdebris02": {
            "width": 1.5139321088790894,
            "length": 1.8574368953704834,
            "height": 0.14900343120098114,
        },
        "static.prop.dirtdebris03": {
            "width": 1.5139321088790894,
            "length": 1.8574368953704834,
            "height": 0.1883699893951416,
        },
        "static.prop.doghouse": {
            "width": 1.2721054553985596,
            "length": 1.0791168212890625,
            "height": 1.0936739444732666,
        },
        "static.prop.dumpster": {
            "width": 1.3943947553634644,
            "length": 2.8662402629852295,
            "height": 2.2673521041870117,
        },
        "static.prop.foodcart": {
            "width": 4.638397693634033,
            "length": 2.2846763134002686,
            "height": 3.573108673095703,
        },
        "static.prop.fountain": {
            "width": 8.401816368103027,
            "length": 8.401811599731445,
            "height": 6.924410343170166,
        },
        "static.prop.garbage01": {
            "width": 0.05342775210738182,
            "length": 0.053427763283252716,
            "height": 0.08676455914974213,
        },
        "static.prop.garbage02": {
            "width": 0.05342775210738182,
            "length": 0.05342775210738182,
            "height": 0.009213896468281746,
        },
        "static.prop.garbage03": {
            "width": 0.05944278463721275,
            "length": 0.059658296406269073,
            "height": 0.07927705347537994,
        },
        "static.prop.garbage04": {
            "width": 0.05568109452724457,
            "length": 0.05727477744221687,
            "height": 0.06884017586708069,
        },
        "static.prop.garbage05": {
            "width": 0.23253268003463745,
            "length": 0.23187801241874695,
            "height": 0.05761278420686722,
        },
        "static.prop.garbage06": {
            "width": 0.2133311778306961,
            "length": 0.3661772906780243,
            "height": 0.07683420181274414,
        },
        "static.prop.gardenlamp": {
            "width": 0.28122183680534363,
            "length": 0.2972412407398224,
            "height": 0.6478500366210938,
        },
        "static.prop.gnome": {
            "width": 0.37559059262275696,
            "length": 0.36661627888679504,
            "height": 0.8829975724220276,
        },
        "static.prop.guitarcase": {
            "width": 0.1251685619354248,
            "length": 1.242393970489502,
            "height": 0.4944811165332794,
        },
        "static.prop.haybale": {
            "width": 1.1933469772338867,
            "length": 1.247710943222046,
            "height": 1.1949762105941772,
        },
        "static.prop.kiosk_01": {
            "width": 1.6540685892105103,
            "length": 1.6252474784851074,
            "height": 3.2453691959381104,
        },
        "static.prop.mailbox": {
            "width": 0.43842461705207825,
            "length": 0.526629626750946,
            "height": 1.1572810411453247,
        },
        "static.prop.maptable": {
            "width": 0.6170762777328491,
            "length": 1.5732090473175049,
            "height": 1.1810814142227173,
        },
        "static.prop.mobile": {
            "width": 0.007261085323989391,
            "length": 0.08048340678215027,
            "height": 0.16641773283481598,
        },
        "static.prop.motorhelmet": {
            "width": 0.2592744827270508,
            "length": 0.20041197538375854,
            "height": 0.25808918476104736,
        },
        "static.prop.pergola": {
            "width": 5.085033893585205,
            "length": 5.029393672943115,
            "height": 3.541531562805176,
        },
        "static.prop.plantpot01": {
            "width": 0.4757719337940216,
            "length": 1.08914053440094,
            "height": 0.5573238730430603,
        },
        "static.prop.plantpot02": {
            "width": 0.43072694540023804,
            "length": 1.0511938333511353,
            "height": 0.48692914843559265,
        },
        "static.prop.plantpot03": {
            "width": 0.43072694540023804,
            "length": 1.0511937141418457,
            "height": 0.4869290888309479,
        },
        "static.prop.plantpot04": {
            "width": 0.7366006970405579,
            "length": 4.967305660247803,
            "height": 0.12478026747703552,
        },
        "static.prop.plantpot05": {
            "width": 0.4757719337940216,
            "length": 0.4946027398109436,
            "height": 0.5573238730430603,
        },
        "static.prop.plantpot06": {
            "width": 1.5932812690734863,
            "length": 1.5932812690734863,
            "height": 0.84954833984375,
        },
        "static.prop.plantpot07": {
            "width": 0.5304248929023743,
            "length": 0.5304248929023743,
            "height": 0.4888741672039032,
        },
        "static.prop.plasticbag": {
            "width": 0.4025624990463257,
            "length": 0.3109833598136902,
            "height": 0.5523712635040283,
        },
        "static.prop.plasticchair": {
            "width": 0.7504488825798035,
            "length": 0.7304753661155701,
            "height": 1.2713558673858643,
        },
        "static.prop.plastictable": {
            "width": 2.4822070598602295,
            "length": 2.4822070598602295,
            "height": 2.479797840118408,
        },
        "static.prop.platformgarbage01": {
            "width": 1.7114051580429077,
            "length": 3.6332411766052246,
            "height": 0.33158016204833984,
        },
        "static.prop.purse": {
            "width": 0.3326959013938904,
            "length": 0.15921518206596375,
            "height": 0.5852110385894775,
        },
        "static.prop.recyclecardboard": {
            "width": 2.3846962451934814,
            "length": 2.5890233516693115,
            "height": 2.0096852779388428,
        },
        "static.prop.recycleglass": {
            "width": 1.7764321565628052,
            "length": 1.0195367336273193,
            "height": 1.943043828010559,
        },
        "static.prop.recycleorganic": {
            "width": 1.7764321565628052,
            "length": 1.0195363759994507,
            "height": 1.943043828010559,
        },
        "static.prop.recycleplastic": {
            "width": 1.9872466325759888,
            "length": 2.5890233516693115,
            "height": 2.188204765319824,
        },
        "static.prop.shoppingbag": {
            "width": 0.2432928830385208,
            "length": 0.47446563839912415,
            "height": 0.6006307601928711,
        },
        "static.prop.shoppingcart": {
            "width": 1.207547903060913,
            "length": 0.6694015264511108,
            "height": 1.0793083906173706,
        },
        "static.prop.shoppingtrolley": {
            "width": 0.33110183477401733,
            "length": 0.5055894255638123,
            "height": 1.0936459302902222,
        },
        "static.prop.streetbarrier": {
            "width": 0.3716789782047272,
            "length": 1.2149077653884888,
            "height": 1.069164514541626,
        },
        "static.prop.streetfountain": {
            "width": 0.9839538335800171,
            "length": 0.405456006526947,
            "height": 1.0484803915023804,
        },
        "static.prop.streetsign": {
            "width": 0.1252390295267105,
            "length": 1.0643447637557983,
            "height": 2.154392957687378,
        },
        "static.prop.streetsign01": {
            "width": 0.3029319941997528,
            "length": 2.470391273498535,
            "height": 3.8779332637786865,
        },
        "static.prop.streetsign04": {
            "width": 0.12483911216259003,
            "length": 1.1708152294158936,
            "height": 2.7728850841522217,
        },
        "static.prop.swing": {
            "width": 1.5768225193023682,
            "length": 4.105227947235107,
            "height": 2.5724740028381348,
        },
        "static.prop.swingcouch": {
            "width": 2.362380266189575,
            "length": 1.357409954071045,
            "height": 2.2493832111358643,
        },
        "static.prop.trafficcone01": {
            "width": 0.9552291035652161,
            "length": 0.955228865146637,
            "height": 1.2336182594299316,
        },
        "static.prop.trafficcone02": {
            "width": 0.3945564925670624,
            "length": 0.455596923828125,
            "height": 1.1829206943511963,
        },
        "static.prop.trafficwarning": {
            "width": 2.8705859184265137,
            "length": 2.373429536819458,
            "height": 3.569523334503174,
        },
        "static.prop.trampoline": {
            "width": 4.131584644317627,
            "length": 4.131584644317627,
            "height": 2.801903247833252,
        },
        "static.prop.trashbag": {
            "width": 0.42955997586250305,
            "length": 0.3779701590538025,
            "height": 0.702082097530365,
        },
        "static.prop.trashcan01": {
            "width": 0.37456512451171875,
            "length": 0.5437172055244446,
            "height": 1.2140138149261475,
        },
        "static.prop.trashcan02": {
            "width": 0.6696233749389648,
            "length": 0.7931143045425415,
            "height": 1.0590858459472656,
        },
        "static.prop.trashcan03": {
            "width": 0.7573251724243164,
            "length": 0.7565627098083496,
            "height": 0.9889887571334839,
        },
        "static.prop.trashcan04": {
            "width": 0.5135547518730164,
            "length": 0.5238340497016907,
            "height": 1.0454455614089966,
        },
        "static.prop.travelcase": {
            "width": 0.3276098966598511,
            "length": 0.5673347115516663,
            "height": 1.262577772140503,
        },
        "static.prop.vendingmachine": {
            "width": 1.102043867111206,
            "length": 0.8728882670402527,
            "height": 2.1082382202148438,
        },
        "static.prop.warningaccident": {
            "width": 1.055053472518921,
            "length": 1.3050163984298706,
            "height": 1.8602019548416138,
        },
        "static.prop.warningconstruction": {
            "width": 1.055053472518921,
            "length": 1.3050163984298706,
            "height": 1.8602019548416138,
        },
        "vehicle.ambulance.ford": {
            "width": 2.3511743545532227,
            "length": 6.356935024261475,
            "height": 2.431001663208008,
        },
        "vehicle.carlacola.actors": {
            "width": 2.9117815494537354,
            "length": 8.003673553466797,
            "height": 4.054566383361816,
        },
        "vehicle.dodge.charger": {
            "width": 1.8809516429901123,
            "length": 5.006043434143066,
            "height": 1.540313720703125,
        },
        "vehicle.dodgecop.charger": {
            "width": 1.9244433641433716,
            "length": 5.236761569976807,
            "height": 1.6439720392227173,
        },
        "vehicle.firetruck.actors": {
            "width": 2.90128493309021,
            "length": 8.579916000366211,
            "height": 3.8267812728881836,
        },
        "vehicle.fuso.mitsubishi": {
            "width": 3.927680492401123,
            "length": 10.174287796020508,
            "height": 4.241000652313232,
        },
        "vehicle.lincoln.mkz": {
            "width": 1.8356460332870483,
            "length": 4.891970157623291,
            "height": 1.5241189002990723,
        },
        "vehicle.mini.cooper": {
            "width": 2.0955777168273926,
            "length": 4.55255126953125,
            "height": 1.7724559307098389,
        },
        "vehicle.nissan.patrol": {
            "width": 2.1466238498687744,
            "length": 5.591163635253906,
            "height": 2.05902099609375,
        },
        "vehicle.sprinter.mercedes": {
            "width": 1.988406777381897,
            "length": 5.9151387214660645,
            "height": 2.7260189056396484,
        },
        "vehicle.taxi.ford": {
            "width": 1.7890609502792358,
            "length": 5.354491233825684,
            "height": 1.575230598449707,
        },
        "walker.pedestrian.0015": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8399999141693115,
        },
        "walker.pedestrian.0016": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8499999046325684,
        },
        "walker.pedestrian.0017": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8499999046325684,
        },
        "walker.pedestrian.0018": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8399999141693115,
        },
        "walker.pedestrian.0019": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8499999046325684,
        },
        "walker.pedestrian.0020": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8399999141693115,
        },
        "walker.pedestrian.0021": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8399999141693115,
        },
        "walker.pedestrian.0022": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8499999046325684,
        },
        "walker.pedestrian.0023": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8499999046325684,
        },
        "walker.pedestrian.0024": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8399999141693115,
        },
        "walker.pedestrian.0025": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8499999046325684,
        },
        "walker.pedestrian.0026": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8399999141693115,
        },
        "walker.pedestrian.0027": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8399999141693115,
        },
        "walker.pedestrian.0028": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8399999141693115,
        },
        "walker.pedestrian.0029": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8399999141693115,
        },
        "walker.pedestrian.0030": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8399999141693115,
        },
        "walker.pedestrian.0031": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8399999141693115,
        },
        "walker.pedestrian.0032": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8399999141693115,
        },
        "walker.pedestrian.0033": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8399999141693115,
        },
        "walker.pedestrian.0034": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8700000047683716,
        },
        "walker.pedestrian.0035": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.9149999618530273,
        },
        "walker.pedestrian.0036": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.9199999570846558,
        },
        "walker.pedestrian.0037": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.9199999570846558,
        },
        "walker.pedestrian.0038": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8700000047683716,
        },
        "walker.pedestrian.0039": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8499999046325684,
        },
        "walker.pedestrian.0040": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8399999141693115,
        },
        "walker.pedestrian.0041": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8399999141693115,
        },
        "walker.pedestrian.0042": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8299999237060547,
        },
        "walker.pedestrian.0043": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8299999237060547,
        },
        "walker.pedestrian.0044": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8299999237060547,
        },
        "walker.pedestrian.0045": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8499999046325684,
        },
        "walker.pedestrian.0046": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8499999046325684,
        },
        "walker.pedestrian.0047": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8499999046325684,
        },
        "walker.pedestrian.0048": {
            "width": 0.5,
            "length": 0.5,
            "height": 1.1100000143051147,
        },
        "walker.pedestrian.0049": {
            "width": 0.5,
            "length": 0.5,
            "height": 1.1100000143051147,
        },
        "walker.pedestrian.0050": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.9049999713897705,
        },
        "walker.pedestrian.0051": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.9049999713897705,
        },
    },
    "0.9.16": {
        "static.prop.advertisement": {
            "width": 0.28684958815574646,
            "length": 1.549233317375183,
            "height": 2.442812442779541,
        },
        "static.prop.aporosatree": {
            "width": 12.584906578063965,
            "length": 12.958136558532715,
            "height": 15.63304615020752,
        },
        "static.prop.atm": {
            "width": 0.8738368153572083,
            "length": 0.7131599187850952,
            "height": 2.2757811546325684,
        },
        "static.prop.barbeque": {
            "width": 0.49884626269340515,
            "length": 0.9456468820571899,
            "height": 1.264062523841858,
        },
        "static.prop.barrel": {
            "width": 0.47143638134002686,
            "length": 0.4596165418624878,
            "height": 0.8067187070846558,
        },
        "static.prop.bench01": {
            "width": 0.735850989818573,
            "length": 1.5440508127212524,
            "height": 0.9697656035423279,
        },
        "static.prop.bench02": {
            "width": 0.5445890426635742,
            "length": 1.5636827945709229,
            "height": 0.5090624690055847,
        },
        "static.prop.bench03": {
            "width": 0.5126523971557617,
            "length": 1.58349609375,
            "height": 0.5126562118530273,
        },
        "static.prop.bike helmet": {
            "width": 0.2770470678806305,
            "length": 0.18554961681365967,
            "height": 0.1713281273841858,
        },
        "static.prop.bin": {
            "width": 0.6381588578224182,
            "length": 0.548203706741333,
            "height": 1.0591405630111694,
        },
        "static.prop.box01": {
            "width": 0.6521967053413391,
            "length": 0.6521967649459839,
            "height": 0.6915624737739563,
        },
        "static.prop.box02": {
            "width": 0.648569643497467,
            "length": 0.648569643497467,
            "height": 0.6485937237739563,
        },
        "static.prop.box03": {
            "width": 0.6700571775436401,
            "length": 0.6608889102935791,
            "height": 0.6485937237739563,
        },
        "static.prop.briefcase": {
            "width": 0.18117640912532806,
            "length": 0.543353796005249,
            "height": 0.43281248211860657,
        },
        "static.prop.brokentile01": {
            "width": 0.1760428547859192,
            "length": 0.13444174826145172,
            "height": 0.006562499795109034,
        },
        "static.prop.brokentile02": {
            "width": 0.15229617059230804,
            "length": 0.20967696607112885,
            "height": 0.007578124757856131,
        },
        "static.prop.brokentile03": {
            "width": 0.1449233442544937,
            "length": 0.15445251762866974,
            "height": 0.008515625260770321,
        },
        "static.prop.brokentile04": {
            "width": 0.12647496163845062,
            "length": 0.12972931563854218,
            "height": 0.01015624962747097,
        },
        "static.prop.busstop": {
            "width": 1.893609881401062,
            "length": 3.8760783672332764,
            "height": 2.738906145095825,
        },
        "static.prop.busstoplb": {
            "width": 3.8760783672332764,
            "length": 1.893609881401062,
            "height": 2.738906145095825,
        },
        "static.prop.calibrator": {
            "width": 0.690200924873352,
            "length": 1.5475953817367554,
            "height": 0.20195311307907104,
        },
        "static.prop.chainbarrier": {
            "width": 0.23859326541423798,
            "length": 1.4941967725753784,
            "height": 0.8887499570846558,
        },
        "static.prop.chainbarrierend": {
            "width": 0.23859326541423798,
            "length": 0.23859328031539917,
            "height": 0.8887499570846558,
        },
        "static.prop.clothcontainer": {
            "width": 1.8865405321121216,
            "length": 1.2394330501556396,
            "height": 1.810156226158142,
        },
        "static.prop.clothesline": {
            "width": 1.1180102825164795,
            "length": 6.652331352233887,
            "height": 2.0610156059265137,
        },
        "static.prop.coconutpalm": {
            "width": 8.885843276977539,
            "length": 8.603760719299316,
            "height": 20.31476593017578,
        },
        "static.prop.colacan": {
            "width": 0.07167824357748032,
            "length": 0.06817007064819336,
            "height": 0.10953124612569809,
        },
        "static.prop.constructioncone": {
            "width": 0.3440696597099304,
            "length": 0.3440696597099304,
            "height": 0.5857812166213989,
        },
        "static.prop.container": {
            "width": 1.0124343633651733,
            "length": 1.9318325519561768,
            "height": 1.7142187356948853,
        },
        "static.prop.creasedbox01": {
            "width": 0.7129369974136353,
            "length": 0.8087886571884155,
            "height": 0.11781249940395355,
        },
        "static.prop.creasedbox02": {
            "width": 0.7129369974136353,
            "length": 1.6332061290740967,
            "height": 0.2116406261920929,
        },
        "static.prop.creasedbox03": {
            "width": 0.7129369974136353,
            "length": 1.6687225103378296,
            "height": 0.021718749776482582,
        },
        "static.prop.cypresstree": {
            "width": 4.628193378448486,
            "length": 4.6099138259887695,
            "height": 15.678828239440918,
        },
        "static.prop.dirtdebris01": {
            "width": 1.5139321088790894,
            "length": 1.8574368953704834,
            "height": 0.1489843726158142,
        },
        "static.prop.dirtdebris02": {
            "width": 1.5139321088790894,
            "length": 1.8574368953704834,
            "height": 0.1489843726158142,
        },
        "static.prop.dirtdebris03": {
            "width": 1.5139321088790894,
            "length": 1.8574368953704834,
            "height": 0.1883593648672104,
        },
        "static.prop.doghouse": {
            "width": 1.2721054553985596,
            "length": 1.0791168212890625,
            "height": 1.0936717987060547,
        },
        "static.prop.foodcart": {
            "width": 4.638397693634033,
            "length": 2.2846763134002686,
            "height": 3.573124885559082,
        },
        "static.prop.fountain": {
            "width": 8.401816368103027,
            "length": 8.401811599731445,
            "height": 6.924375057220459,
        },
        "static.prop.garbage01": {
            "width": 0.05342773348093033,
            "length": 0.05342777073383331,
            "height": 0.08679687231779099,
        },
        "static.prop.garbage02": {
            "width": 0.05342775210738182,
            "length": 0.05342775210738182,
            "height": 0.009218749590218067,
        },
        "static.prop.garbage03": {
            "width": 0.05944278463721275,
            "length": 0.05965827777981758,
            "height": 0.07929687201976776,
        },
        "static.prop.garbage04": {
            "width": 0.055681075900793076,
            "length": 0.05727477744221687,
            "height": 0.06882812082767487,
        },
        "static.prop.garbage05": {
            "width": 0.23253268003463745,
            "length": 0.23187801241874695,
            "height": 0.05757812410593033,
        },
        "static.prop.garbage06": {
            "width": 0.21333114802837372,
            "length": 0.3661772906780243,
            "height": 0.07679687440395355,
        },
        "static.prop.gardenlamp": {
            "width": 0.28122183680534363,
            "length": 0.2972412407398224,
            "height": 0.6478124856948853,
        },
        "static.prop.glasscontainer": {
            "width": 2.0087928771972656,
            "length": 1.9547909498214722,
            "height": 1.7884374856948853,
        },
        "static.prop.gnome": {
            "width": 0.37559059262275696,
            "length": 0.36661627888679504,
            "height": 0.8829687237739563,
        },
        "static.prop.guitarcase": {
            "width": 0.1251685619354248,
            "length": 1.242393970489502,
            "height": 0.494453102350235,
        },
        "static.prop.haybale": {
            "width": 1.1933469772338867,
            "length": 1.247710943222046,
            "height": 1.1949999332427979,
        },
        "static.prop.haybalelb": {
            "width": 1.1933469772338867,
            "length": 1.247710943222046,
            "height": 1.1949999332427979,
        },
        "static.prop.ironplank": {
            "width": 1.1743810176849365,
            "length": 1.45187246799469,
            "height": 0.020546874031424522,
        },
        "static.prop.kiosk_01": {
            "width": 1.6540685892105103,
            "length": 1.6252474784851074,
            "height": 3.2453906536102295,
        },
        "static.prop.mailbox": {
            "width": 0.43842464685440063,
            "length": 0.526629626750946,
            "height": 1.157265543937683,
        },
        "static.prop.maptable": {
            "width": 0.6170762777328491,
            "length": 1.5732090473175049,
            "height": 1.181093692779541,
        },
        "static.prop.mobile": {
            "width": 0.007261085323989391,
            "length": 0.08048340678215027,
            "height": 0.16640624403953552,
        },
        "static.prop.motorhelmet": {
            "width": 0.2592744827270508,
            "length": 0.20041197538375854,
            "height": 0.2581250071525574,
        },
        "static.prop.pergola": {
            "width": 5.085033893585205,
            "length": 5.029393672943115,
            "height": 3.54156231880188,
        },
        "static.prop.plantpot01": {
            "width": 0.4757719337940216,
            "length": 1.08914053440094,
            "height": 0.5573437213897705,
        },
        "static.prop.plantpot02": {
            "width": 0.43072694540023804,
            "length": 1.0511938333511353,
            "height": 0.48695310950279236,
        },
        "static.prop.plantpot03": {
            "width": 0.43072694540023804,
            "length": 1.0511937141418457,
            "height": 0.48695310950279236,
        },
        "static.prop.plantpot04": {
            "width": 0.7366006970405579,
            "length": 4.967305660247803,
            "height": 0.12476561963558197,
        },
        "static.prop.plantpot05": {
            "width": 0.40022480487823486,
            "length": 0.40022480487823486,
            "height": 0.2800000011920929,
        },
        "static.prop.plantpot06": {
            "width": 1.5932812690734863,
            "length": 1.5932812690734863,
            "height": 0.8495312333106995,
        },
        "static.prop.plantpot07": {
            "width": 0.4757719337940216,
            "length": 0.4946027398109436,
            "height": 0.5573437213897705,
        },
        "static.prop.plantpot08": {
            "width": 0.5304248929023743,
            "length": 0.5304248929023743,
            "height": 0.48890623450279236,
        },
        "static.prop.plasticbag": {
            "width": 0.4025624990463257,
            "length": 0.3109833598136902,
            "height": 0.5523437261581421,
        },
        "static.prop.plasticchair": {
            "width": 0.7504488825798035,
            "length": 0.7304753661155701,
            "height": 1.271328091621399,
        },
        "static.prop.plastictable": {
            "width": 2.482203245162964,
            "length": 2.482203245162964,
            "height": 2.4797656536102295,
        },
        "static.prop.platformgarbage01": {
            "width": 1.7114051580429077,
            "length": 3.6332411766052246,
            "height": 0.33156248927116394,
        },
        "static.prop.purse": {
            "width": 0.3326959013938904,
            "length": 0.15921516716480255,
            "height": 0.5852343440055847,
        },
        "static.prop.shoppingbag": {
            "width": 0.2432928830385208,
            "length": 0.47446563839912415,
            "height": 0.6006249785423279,
        },
        "static.prop.shoppingcart": {
            "width": 1.207547903060913,
            "length": 0.6694015264511108,
            "height": 1.0792968273162842,
        },
        "static.prop.shoppingtrolley": {
            "width": 0.33110183477401733,
            "length": 0.5055894255638123,
            "height": 1.0936717987060547,
        },
        "static.prop.slide": {
            "width": 4.122486591339111,
            "length": 0.9425268173217773,
            "height": 1.9766405820846558,
        },
        "static.prop.streetbarrier": {
            "width": 0.3716789782047272,
            "length": 1.2149077653884888,
            "height": 1.0691405534744263,
        },
        "static.prop.streetfountain": {
            "width": 0.6702813506126404,
            "length": 0.2869437038898468,
            "height": 1.259374976158142,
        },
        "static.prop.streetsign": {
            "width": 0.1252390295267105,
            "length": 1.0643447637557983,
            "height": 2.154374837875366,
        },
        "static.prop.streetsign01": {
            "width": 0.3029319941997528,
            "length": 2.470391273498535,
            "height": 3.8779685497283936,
        },
        "static.prop.streetsign04": {
            "width": 0.12483911216259003,
            "length": 1.1708152294158936,
            "height": 2.772890567779541,
        },
        "static.prop.swing": {
            "width": 1.5768225193023682,
            "length": 4.105227947235107,
            "height": 2.572499990463257,
        },
        "static.prop.swingcouch": {
            "width": 2.362380266189575,
            "length": 1.357409954071045,
            "height": 2.2493748664855957,
        },
        "static.prop.table": {
            "width": 2.1518361568450928,
            "length": 2.1562821865081787,
            "height": 0.846484363079071,
        },
        "static.prop.trafficcone01": {
            "width": 0.8821874856948853,
            "length": 0.8828125,
            "height": 1.1332812309265137,
        },
        "static.prop.trafficcone02": {
            "width": 0.3945564925670624,
            "length": 0.455596923828125,
            "height": 1.1828906536102295,
        },
        "static.prop.trafficwarning": {
            "width": 2.8705859184265137,
            "length": 2.373429536819458,
            "height": 3.569531202316284,
        },
        "static.prop.trampoline": {
            "width": 4.131584644317627,
            "length": 4.131584644317627,
            "height": 2.801874876022339,
        },
        "static.prop.trashbag": {
            "width": 0.42955997586250305,
            "length": 0.3779701590538025,
            "height": 0.7021093368530273,
        },
        "static.prop.trashcan01": {
            "width": 0.5678514838218689,
            "length": 0.6352845430374146,
            "height": 0.8482031226158142,
        },
        "static.prop.trashcan02": {
            "width": 0.5135547518730164,
            "length": 0.5238340497016907,
            "height": 1.0454686880111694,
        },
        "static.prop.trashcan03": {
            "width": 0.5453212857246399,
            "length": 0.6293092370033264,
            "height": 0.93359375,
        },
        "static.prop.trashcan04": {
            "width": 0.5803966522216797,
            "length": 0.7887265086174011,
            "height": 1.0163280963897705,
        },
        "static.prop.trashcan05": {
            "width": 0.5803966522216797,
            "length": 0.7887265086174011,
            "height": 1.0163280963897705,
        },
        "static.prop.travelcase": {
            "width": 0.3276098966598511,
            "length": 0.5673347115516663,
            "height": 1.2625781297683716,
        },
        "static.prop.vendingmachine": {
            "width": 1.102043867111206,
            "length": 0.8728882670402527,
            "height": 2.108203172683716,
        },
        "static.prop.warningaccident": {
            "width": 1.055053472518921,
            "length": 1.3050163984298706,
            "height": 1.8602343797683716,
        },
        "static.prop.warningconstruction": {
            "width": 1.055053472518921,
            "length": 1.3050163984298706,
            "height": 1.8602343797683716,
        },
        "static.prop.wateringcan": {
            "width": 0.19403739273548126,
            "length": 0.07035235315561295,
            "height": 0.13484375178813934,
        },
        "vehicle.audi.a2": {
            "width": 1.788678526878357,
            "length": 3.705369472503662,
            "height": 1.549050211906433,
        },
        "vehicle.audi.etron": {
            "width": 2.0327565670013428,
            "length": 4.855708599090576,
            "height": 1.6493593454360962,
        },
        "vehicle.audi.tt": {
            "width": 1.9941171407699585,
            "length": 4.181210041046143,
            "height": 1.385296106338501,
        },
        "vehicle.bh.crossbike": {
            "width": 0.8659406304359436,
            "length": 1.5093227624893188,
            "height": 1.6123536825180054,
        },
        "vehicle.bmw.grandtourer": {
            "width": 2.241713285446167,
            "length": 4.611005783081055,
            "height": 1.6672759056091309,
        },
        "vehicle.carlamotors.carlacola": {
            "width": 2.6269896030426025,
            "length": 5.203838348388672,
            "height": 2.467444658279419,
        },
        "vehicle.carlamotors.european_hgv": {
            "width": 2.8910882472991943,
            "length": 7.935710430145264,
            "height": 3.4619433879852295,
        },
        "vehicle.carlamotors.firetruck": {
            "width": 2.8910882472991943,
            "length": 8.46804141998291,
            "height": 3.8274123668670654,
        },
        "vehicle.chevrolet.impala": {
            "width": 2.033202886581421,
            "length": 5.357479572296143,
            "height": 1.4106587171554565,
        },
        "vehicle.citroen.c3": {
            "width": 1.8508483171463013,
            "length": 3.987684965133667,
            "height": 1.6171095371246338,
        },
        "vehicle.diamondback.century": {
            "width": 0.5824381709098816,
            "length": 1.6562436819076538,
            "height": 1.6197669506072998,
        },
        "vehicle.dodge.charger_2020": {
            "width": 1.8816219568252563,
            "length": 5.0078253746032715,
            "height": 1.5347249507904053,
        },
        "vehicle.dodge.charger_police": {
            "width": 2.0384011268615723,
            "length": 4.974244117736816,
            "height": 1.5421181917190552,
        },
        "vehicle.dodge.charger_police_2020": {
            "width": 1.9297595024108887,
            "length": 5.237514495849609,
            "height": 1.638383150100708,
        },
        "vehicle.ford.ambulance": {
            "width": 2.3511743545532227,
            "length": 6.36564302444458,
            "height": 2.431375741958618,
        },
        "vehicle.ford.crown": {
            "width": 1.8007241487503052,
            "length": 5.365678787231445,
            "height": 1.5749659538269043,
        },
        "vehicle.ford.mustang": {
            "width": 1.894826889038086,
            "length": 4.717525005340576,
            "height": 1.300939917564392,
        },
        "vehicle.gazelle.omafiets": {
            "width": 0.6590427160263062,
            "length": 1.843441367149353,
            "height": 1.7758288383483887,
        },
        "vehicle.harley-davidson.low_rider": {
            "width": 0.7662330269813538,
            "length": 2.350175619125366,
            "height": 1.6494941711425781,
        },
        "vehicle.jeep.wrangler_rubicon": {
            "width": 1.9051965475082397,
            "length": 3.866220712661743,
            "height": 1.8779358863830566,
        },
        "vehicle.kawasaki.ninja": {
            "width": 0.7969123125076294,
            "length": 2.043684244155884,
            "height": 1.523191213607788,
        },
        "vehicle.lincoln.mkz_2017": {
            "width": 2.128324270248413,
            "length": 4.901683330535889,
            "height": 1.5107464790344238,
        },
        "vehicle.lincoln.mkz_2020": {
            "width": 1.8367133140563965,
            "length": 4.89238166809082,
            "height": 1.490277647972107,
        },
        "vehicle.mercedes.coupe": {
            "width": 2.1515462398529053,
            "length": 5.0267767906188965,
            "height": 1.6506516933441162,
        },
        "vehicle.mercedes.coupe_2020": {
            "width": 1.8118125200271606,
            "length": 4.673638820648193,
            "height": 1.441947340965271,
        },
        "vehicle.mercedes.sprinter": {
            "width": 1.9884328842163086,
            "length": 5.91519021987915,
            "height": 2.560655355453491,
        },
        "vehicle.micro.microlino": {
            "width": 1.4809197187423706,
            "length": 2.2072951793670654,
            "height": 1.3760247230529785,
        },
        "vehicle.mini.cooper_s": {
            "width": 1.970275640487671,
            "length": 3.805800199508667,
            "height": 1.4750301837921143,
        },
        "vehicle.mini.cooper_s_2021": {
            "width": 2.097072124481201,
            "length": 4.552699089050293,
            "height": 1.7671663761138916,
        },
        "vehicle.mitsubishi.fusorosa": {
            "width": 3.9441518783569336,
            "length": 10.272685050964355,
            "height": 4.252848148345947,
        },
        "vehicle.nissan.micra": {
            "width": 1.845113754272461,
            "length": 3.633375883102417,
            "height": 1.5012825727462769,
        },
        "vehicle.nissan.patrol": {
            "width": 1.9315929412841797,
            "length": 4.6045098304748535,
            "height": 1.8548461198806763,
        },
        "vehicle.nissan.patrol_2021": {
            "width": 2.1499669551849365,
            "length": 5.565828800201416,
            "height": 2.045147180557251,
        },
        "vehicle.seat.leon": {
            "width": 1.8161858320236206,
            "length": 4.1928300857543945,
            "height": 1.4738311767578125,
        },
        "vehicle.tesla.cybertruck": {
            "width": 2.3895740509033203,
            "length": 6.273553371429443,
            "height": 2.098191261291504,
        },
        "vehicle.tesla.model3": {
            "width": 2.163450002670288,
            "length": 4.791779518127441,
            "height": 1.4876600503921509,
        },
        "vehicle.toyota.prius": {
            "width": 2.006814479827881,
            "length": 4.513522624969482,
            "height": 1.5248334407806396,
        },
        "vehicle.vespa.zx125": {
            "width": 0.8659406304359436,
            "length": 1.8171066045761108,
            "height": 1.5900908708572388,
        },
        "vehicle.volkswagen.t2": {
            "width": 2.069315195083618,
            "length": 4.4804368019104,
            "height": 2.0377919673919678,
        },
        "vehicle.volkswagen.t2_2021": {
            "width": 1.77456533908844,
            "length": 4.442183971405029,
            "height": 1.9872066974639893,
        },
        "vehicle.yamaha.yzf": {
            "width": 0.8659172654151917,
            "length": 2.1907684803009033,
            "height": 1.530381441116333,
        },
        "walker.pedestrian.0001": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0002": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0003": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0004": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0005": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0006": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0007": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0008": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0009": {
            "width": 0.5,
            "length": 0.5,
            "height": 1.100000023841858,
        },
        "walker.pedestrian.0010": {
            "width": 0.5,
            "length": 0.5,
            "height": 1.100000023841858,
        },
        "walker.pedestrian.0011": {
            "width": 0.5,
            "length": 0.5,
            "height": 1.100000023841858,
        },
        "walker.pedestrian.0012": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.2999999523162842,
        },
        "walker.pedestrian.0013": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.2999999523162842,
        },
        "walker.pedestrian.0014": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.2999999523162842,
        },
        "walker.pedestrian.0015": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0016": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0017": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0018": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0019": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0020": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0021": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0022": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0023": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0024": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0025": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0026": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0027": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0028": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0029": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0030": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0031": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0032": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0033": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0034": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0035": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0036": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0037": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0038": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0039": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0040": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0041": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0042": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0043": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0044": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0045": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0046": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0047": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0048": {
            "width": 0.5,
            "length": 0.5,
            "height": 1.100000023841858,
        },
        "walker.pedestrian.0049": {
            "width": 0.5,
            "length": 0.5,
            "height": 1.100000023841858,
        },
        "walker.pedestrian.0050": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0051": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0052": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
    },
    "0.9.15": {
        "static.prop.advertisement": {
            "width": 0.28684958815574646,
            "length": 1.549233317375183,
            "height": 2.442812442779541,
        },
        "static.prop.atm": {
            "width": 0.8738368153572083,
            "length": 0.7131599187850952,
            "height": 2.2757811546325684,
        },
        "static.prop.barbeque": {
            "width": 0.49884626269340515,
            "length": 0.9456468820571899,
            "height": 1.264062523841858,
        },
        "static.prop.barrel": {
            "width": 0.47143638134002686,
            "length": 0.4596165418624878,
            "height": 0.8067187070846558,
        },
        "static.prop.bench01": {
            "width": 0.735850989818573,
            "length": 1.5440508127212524,
            "height": 0.9697656035423279,
        },
        "static.prop.bench02": {
            "width": 0.5445890426635742,
            "length": 1.5636827945709229,
            "height": 0.5090624690055847,
        },
        "static.prop.bench03": {
            "width": 0.5126523971557617,
            "length": 1.58349609375,
            "height": 0.5126562118530273,
        },
        "static.prop.bike helmet": {
            "width": 0.2770470678806305,
            "length": 0.18554961681365967,
            "height": 0.1713281273841858,
        },
        "static.prop.bin": {
            "width": 0.6381588578224182,
            "length": 0.548203706741333,
            "height": 1.0591405630111694,
        },
        "static.prop.box01": {
            "width": 0.6521967053413391,
            "length": 0.6521967649459839,
            "height": 0.6915624737739563,
        },
        "static.prop.box02": {
            "width": 0.648569643497467,
            "length": 0.648569643497467,
            "height": 0.6485937237739563,
        },
        "static.prop.box03": {
            "width": 0.6700571775436401,
            "length": 0.6608889102935791,
            "height": 0.6485937237739563,
        },
        "static.prop.briefcase": {
            "width": 0.18117640912532806,
            "length": 0.543353796005249,
            "height": 0.43281248211860657,
        },
        "static.prop.brokentile01": {
            "width": 0.1760428547859192,
            "length": 0.13444174826145172,
            "height": 0.006562499795109034,
        },
        "static.prop.brokentile02": {
            "width": 0.15229617059230804,
            "length": 0.20967696607112885,
            "height": 0.007578124757856131,
        },
        "static.prop.brokentile03": {
            "width": 0.1449233442544937,
            "length": 0.15445251762866974,
            "height": 0.008515625260770321,
        },
        "static.prop.brokentile04": {
            "width": 0.12647496163845062,
            "length": 0.12972931563854218,
            "height": 0.01015624962747097,
        },
        "static.prop.busstop": {
            "width": 1.893609881401062,
            "length": 3.8760783672332764,
            "height": 2.738906145095825,
        },
        "static.prop.busstoplb": {
            "width": 3.8760783672332764,
            "length": 1.893609881401062,
            "height": 2.738906145095825,
        },
        "static.prop.calibrator": {
            "width": 0.690200924873352,
            "length": 1.5475953817367554,
            "height": 0.20195311307907104,
        },
        "static.prop.chainbarrier": {
            "width": 0.23859326541423798,
            "length": 1.4941967725753784,
            "height": 0.8887499570846558,
        },
        "static.prop.chainbarrierend": {
            "width": 0.23859326541423798,
            "length": 0.23859328031539917,
            "height": 0.8887499570846558,
        },
        "static.prop.clothcontainer": {
            "width": 1.8865405321121216,
            "length": 1.2394330501556396,
            "height": 1.810156226158142,
        },
        "static.prop.clothesline": {
            "width": 1.1180102825164795,
            "length": 6.652331352233887,
            "height": 2.0610156059265137,
        },
        "static.prop.colacan": {
            "width": 0.07167824357748032,
            "length": 0.06817007064819336,
            "height": 0.10953124612569809,
        },
        "static.prop.constructioncone": {
            "width": 0.3440696597099304,
            "length": 0.3440696597099304,
            "height": 0.5857812166213989,
        },
        "static.prop.container": {
            "width": 1.0124343633651733,
            "length": 1.9318325519561768,
            "height": 1.7142187356948853,
        },
        "static.prop.creasedbox01": {
            "width": 0.7129369974136353,
            "length": 0.8087886571884155,
            "height": 0.11781249940395355,
        },
        "static.prop.creasedbox02": {
            "width": 0.7129369974136353,
            "length": 1.6332061290740967,
            "height": 0.2116406261920929,
        },
        "static.prop.creasedbox03": {
            "width": 0.7129369974136353,
            "length": 1.6687225103378296,
            "height": 0.021718749776482582,
        },
        "static.prop.dirtdebris01": {
            "width": 1.5139321088790894,
            "length": 1.8574368953704834,
            "height": 0.1489843726158142,
        },
        "static.prop.dirtdebris02": {
            "width": 1.5139321088790894,
            "length": 1.8574368953704834,
            "height": 0.1489843726158142,
        },
        "static.prop.dirtdebris03": {
            "width": 1.5139321088790894,
            "length": 1.8574368953704834,
            "height": 0.1883593648672104,
        },
        "static.prop.doghouse": {
            "width": 1.2721054553985596,
            "length": 1.0791168212890625,
            "height": 1.0936717987060547,
        },
        "static.prop.foodcart": {
            "width": 4.638397693634033,
            "length": 2.2846763134002686,
            "height": 3.573124885559082,
        },
        "static.prop.fountain": {
            "width": 8.401816368103027,
            "length": 8.401811599731445,
            "height": 6.924375057220459,
        },
        "static.prop.garbage01": {
            "width": 0.05342773348093033,
            "length": 0.05342777073383331,
            "height": 0.08679687231779099,
        },
        "static.prop.garbage02": {
            "width": 0.05342775210738182,
            "length": 0.05342775210738182,
            "height": 0.009218749590218067,
        },
        "static.prop.garbage03": {
            "width": 0.05944278463721275,
            "length": 0.05965827777981758,
            "height": 0.07929687201976776,
        },
        "static.prop.garbage04": {
            "width": 0.055681075900793076,
            "length": 0.05727477744221687,
            "height": 0.06882812082767487,
        },
        "static.prop.garbage05": {
            "width": 0.23253268003463745,
            "length": 0.23187801241874695,
            "height": 0.05757812410593033,
        },
        "static.prop.garbage06": {
            "width": 0.21333114802837372,
            "length": 0.3661772906780243,
            "height": 0.07679687440395355,
        },
        "static.prop.gardenlamp": {
            "width": 0.28122183680534363,
            "length": 0.2972412407398224,
            "height": 0.6478124856948853,
        },
        "static.prop.glasscontainer": {
            "width": 2.0087928771972656,
            "length": 1.9547909498214722,
            "height": 1.7884374856948853,
        },
        "static.prop.gnome": {
            "width": 0.37559059262275696,
            "length": 0.36661627888679504,
            "height": 0.8829687237739563,
        },
        "static.prop.guitarcase": {
            "width": 0.1251685619354248,
            "length": 1.242393970489502,
            "height": 0.494453102350235,
        },
        "static.prop.haybale": {
            "width": 1.1933469772338867,
            "length": 1.247710943222046,
            "height": 1.1949999332427979,
        },
        "static.prop.haybalelb": {
            "width": 1.1933469772338867,
            "length": 1.247710943222046,
            "height": 1.1949999332427979,
        },
        "static.prop.ironplank": {
            "width": 1.1743810176849365,
            "length": 1.45187246799469,
            "height": 0.020546874031424522,
        },
        "static.prop.kiosk_01": {
            "width": 1.6540685892105103,
            "length": 1.6252474784851074,
            "height": 3.2453906536102295,
        },
        "static.prop.mailbox": {
            "width": 0.43842464685440063,
            "length": 0.526629626750946,
            "height": 1.157265543937683,
        },
        "static.prop.maptable": {
            "width": 0.6170762777328491,
            "length": 1.5732090473175049,
            "height": 1.181093692779541,
        },
        "static.prop.mobile": {
            "width": 0.007261085323989391,
            "length": 0.08048340678215027,
            "height": 0.16640624403953552,
        },
        "static.prop.motorhelmet": {
            "width": 0.2592744827270508,
            "length": 0.20041197538375854,
            "height": 0.2581250071525574,
        },
        "static.prop.pergola": {
            "width": 5.085033893585205,
            "length": 5.029393672943115,
            "height": 3.54156231880188,
        },
        "static.prop.plantpot01": {
            "width": 0.4757719337940216,
            "length": 1.08914053440094,
            "height": 0.5573437213897705,
        },
        "static.prop.plantpot02": {
            "width": 0.43072694540023804,
            "length": 1.0511938333511353,
            "height": 0.48695310950279236,
        },
        "static.prop.plantpot03": {
            "width": 0.43072694540023804,
            "length": 1.0511937141418457,
            "height": 0.48695310950279236,
        },
        "static.prop.plantpot04": {
            "width": 0.7366006970405579,
            "length": 4.967305660247803,
            "height": 0.12476561963558197,
        },
        "static.prop.plantpot05": {
            "width": 0.40022480487823486,
            "length": 0.40022480487823486,
            "height": 0.2800000011920929,
        },
        "static.prop.plantpot06": {
            "width": 1.5932812690734863,
            "length": 1.5932812690734863,
            "height": 0.8495312333106995,
        },
        "static.prop.plantpot07": {
            "width": 0.4757719337940216,
            "length": 0.4946027398109436,
            "height": 0.5573437213897705,
        },
        "static.prop.plantpot08": {
            "width": 0.5304248929023743,
            "length": 0.5304248929023743,
            "height": 0.48890623450279236,
        },
        "static.prop.plasticbag": {
            "width": 0.4025624990463257,
            "length": 0.3109833598136902,
            "height": 0.5523437261581421,
        },
        "static.prop.plasticchair": {
            "width": 0.7504488825798035,
            "length": 0.7304753661155701,
            "height": 1.271328091621399,
        },
        "static.prop.plastictable": {
            "width": 2.482203245162964,
            "length": 2.482203245162964,
            "height": 2.4797656536102295,
        },
        "static.prop.platformgarbage01": {
            "width": 1.7114051580429077,
            "length": 3.6332411766052246,
            "height": 0.33156248927116394,
        },
        "static.prop.purse": {
            "width": 0.3326959013938904,
            "length": 0.15921516716480255,
            "height": 0.5852343440055847,
        },
        "static.prop.shoppingbag": {
            "width": 0.2432928830385208,
            "length": 0.47446563839912415,
            "height": 0.6006249785423279,
        },
        "static.prop.shoppingcart": {
            "width": 1.207547903060913,
            "length": 0.6694015264511108,
            "height": 1.0792968273162842,
        },
        "static.prop.shoppingtrolley": {
            "width": 0.33110183477401733,
            "length": 0.5055894255638123,
            "height": 1.0936717987060547,
        },
        "static.prop.slide": {
            "width": 4.122486591339111,
            "length": 0.9425268173217773,
            "height": 1.9766405820846558,
        },
        "static.prop.streetbarrier": {
            "width": 0.3716789782047272,
            "length": 1.2149077653884888,
            "height": 1.0691405534744263,
        },
        "static.prop.streetfountain": {
            "width": 0.6702813506126404,
            "length": 0.2869437038898468,
            "height": 1.259374976158142,
        },
        "static.prop.streetsign": {
            "width": 0.1252390295267105,
            "length": 1.0643447637557983,
            "height": 2.154374837875366,
        },
        "static.prop.streetsign01": {
            "width": 0.3029319941997528,
            "length": 2.470391273498535,
            "height": 3.8779685497283936,
        },
        "static.prop.streetsign04": {
            "width": 0.12483911216259003,
            "length": 1.1708152294158936,
            "height": 2.772890567779541,
        },
        "static.prop.swing": {
            "width": 1.5768225193023682,
            "length": 4.105227947235107,
            "height": 2.572499990463257,
        },
        "static.prop.swingcouch": {
            "width": 2.362380266189575,
            "length": 1.357409954071045,
            "height": 2.2493748664855957,
        },
        "static.prop.table": {
            "width": 2.1518361568450928,
            "length": 2.1562821865081787,
            "height": 0.846484363079071,
        },
        "static.prop.trafficcone01": {
            "width": 0.8821874856948853,
            "length": 0.8828125,
            "height": 1.1332812309265137,
        },
        "static.prop.trafficcone02": {
            "width": 0.3945564925670624,
            "length": 0.455596923828125,
            "height": 1.1828906536102295,
        },
        "static.prop.trafficwarning": {
            "width": 2.8705859184265137,
            "length": 2.373429536819458,
            "height": 3.569531202316284,
        },
        "static.prop.trampoline": {
            "width": 4.131584644317627,
            "length": 4.131584644317627,
            "height": 2.801874876022339,
        },
        "static.prop.trashbag": {
            "width": 0.42955997586250305,
            "length": 0.3779701590538025,
            "height": 0.7021093368530273,
        },
        "static.prop.trashcan01": {
            "width": 0.5678514838218689,
            "length": 0.6352845430374146,
            "height": 0.8482031226158142,
        },
        "static.prop.trashcan02": {
            "width": 0.5135547518730164,
            "length": 0.5238340497016907,
            "height": 1.0454686880111694,
        },
        "static.prop.trashcan03": {
            "width": 0.5453212857246399,
            "length": 0.6293092370033264,
            "height": 0.93359375,
        },
        "static.prop.trashcan04": {
            "width": 0.5803966522216797,
            "length": 0.7887265086174011,
            "height": 1.0163280963897705,
        },
        "static.prop.trashcan05": {
            "width": 0.5803966522216797,
            "length": 0.7887265086174011,
            "height": 1.0163280963897705,
        },
        "static.prop.travelcase": {
            "width": 0.3276098966598511,
            "length": 0.5673347115516663,
            "height": 1.2625781297683716,
        },
        "static.prop.vendingmachine": {
            "width": 1.102043867111206,
            "length": 0.8728882670402527,
            "height": 2.108203172683716,
        },
        "static.prop.warningaccident": {
            "width": 1.055053472518921,
            "length": 1.3050163984298706,
            "height": 1.8602343797683716,
        },
        "static.prop.warningconstruction": {
            "width": 1.055053472518921,
            "length": 1.3050163984298706,
            "height": 1.8602343797683716,
        },
        "static.prop.wateringcan": {
            "width": 0.19403739273548126,
            "length": 0.07035235315561295,
            "height": 0.13484375178813934,
        },
        "vehicle.audi.a2": {
            "width": 1.788678526878357,
            "length": 3.705369472503662,
            "height": 1.549050211906433,
        },
        "vehicle.audi.etron": {
            "width": 2.0327565670013428,
            "length": 4.855708599090576,
            "height": 1.6493593454360962,
        },
        "vehicle.audi.tt": {
            "width": 1.9941171407699585,
            "length": 4.181210041046143,
            "height": 1.385296106338501,
        },
        "vehicle.bh.crossbike": {
            "width": 0.8659406304359436,
            "length": 1.5093227624893188,
            "height": 1.6123536825180054,
        },
        "vehicle.bmw.grandtourer": {
            "width": 2.241713285446167,
            "length": 4.611005783081055,
            "height": 1.6672759056091309,
        },
        "vehicle.carlamotors.carlacola": {
            "width": 2.6269896030426025,
            "length": 5.203838348388672,
            "height": 2.467444658279419,
        },
        "vehicle.carlamotors.european_hgv": {
            "width": 2.8910882472991943,
            "length": 7.935710430145264,
            "height": 3.4619433879852295,
        },
        "vehicle.carlamotors.firetruck": {
            "width": 2.8910882472991943,
            "length": 8.46804141998291,
            "height": 3.8274123668670654,
        },
        "vehicle.chevrolet.impala": {
            "width": 2.033202886581421,
            "length": 5.357479572296143,
            "height": 1.4106587171554565,
        },
        "vehicle.citroen.c3": {
            "width": 1.8508483171463013,
            "length": 3.987684965133667,
            "height": 1.6171095371246338,
        },
        "vehicle.diamondback.century": {
            "width": 0.5824381709098816,
            "length": 1.6562436819076538,
            "height": 1.6197669506072998,
        },
        "vehicle.dodge.charger_2020": {
            "width": 1.8816219568252563,
            "length": 5.0078253746032715,
            "height": 1.5347249507904053,
        },
        "vehicle.dodge.charger_police": {
            "width": 2.0384011268615723,
            "length": 4.974244117736816,
            "height": 1.5421181917190552,
        },
        "vehicle.dodge.charger_police_2020": {
            "width": 1.9297595024108887,
            "length": 5.237514495849609,
            "height": 1.638383150100708,
        },
        "vehicle.ford.ambulance": {
            "width": 2.3511743545532227,
            "length": 6.36564302444458,
            "height": 2.431375741958618,
        },
        "vehicle.ford.crown": {
            "width": 1.8007241487503052,
            "length": 5.365678787231445,
            "height": 1.5749659538269043,
        },
        "vehicle.ford.mustang": {
            "width": 1.894826889038086,
            "length": 4.717525005340576,
            "height": 1.300939917564392,
        },
        "vehicle.gazelle.omafiets": {
            "width": 0.6590427160263062,
            "length": 1.843441367149353,
            "height": 1.7758288383483887,
        },
        "vehicle.harley-davidson.low_rider": {
            "width": 0.7662330269813538,
            "length": 2.350175619125366,
            "height": 1.6494941711425781,
        },
        "vehicle.jeep.wrangler_rubicon": {
            "width": 1.9051965475082397,
            "length": 3.866220712661743,
            "height": 1.8779358863830566,
        },
        "vehicle.kawasaki.ninja": {
            "width": 0.7969123125076294,
            "length": 2.043684244155884,
            "height": 1.523191213607788,
        },
        "vehicle.lincoln.mkz_2017": {
            "width": 2.128324270248413,
            "length": 4.901683330535889,
            "height": 1.5107464790344238,
        },
        "vehicle.lincoln.mkz_2020": {
            "width": 1.8367133140563965,
            "length": 4.89238166809082,
            "height": 1.490277647972107,
        },
        "vehicle.mercedes.coupe": {
            "width": 2.1515462398529053,
            "length": 5.0267767906188965,
            "height": 1.6506516933441162,
        },
        "vehicle.mercedes.coupe_2020": {
            "width": 1.8118125200271606,
            "length": 4.673638820648193,
            "height": 1.441947340965271,
        },
        "vehicle.mercedes.sprinter": {
            "width": 1.9884328842163086,
            "length": 5.91519021987915,
            "height": 2.560655355453491,
        },
        "vehicle.micro.microlino": {
            "width": 1.4809197187423706,
            "length": 2.2072951793670654,
            "height": 1.3760247230529785,
        },
        "vehicle.mini.cooper_s": {
            "width": 1.970275640487671,
            "length": 3.805800199508667,
            "height": 1.4750301837921143,
        },
        "vehicle.mini.cooper_s_2021": {
            "width": 2.097072124481201,
            "length": 4.552699089050293,
            "height": 1.7671663761138916,
        },
        "vehicle.mitsubishi.fusorosa": {
            "width": 3.9441518783569336,
            "length": 10.272685050964355,
            "height": 4.252848148345947,
        },
        "vehicle.nissan.micra": {
            "width": 1.845113754272461,
            "length": 3.633375883102417,
            "height": 1.5012825727462769,
        },
        "vehicle.nissan.patrol": {
            "width": 1.9315929412841797,
            "length": 4.6045098304748535,
            "height": 1.8548461198806763,
        },
        "vehicle.nissan.patrol_2021": {
            "width": 2.1499669551849365,
            "length": 5.565828800201416,
            "height": 2.045147180557251,
        },
        "vehicle.seat.leon": {
            "width": 1.8161858320236206,
            "length": 4.1928300857543945,
            "height": 1.4738311767578125,
        },
        "vehicle.tesla.cybertruck": {
            "width": 2.3895740509033203,
            "length": 6.273553371429443,
            "height": 2.098191261291504,
        },
        "vehicle.tesla.model3": {
            "width": 2.163450002670288,
            "length": 4.791779518127441,
            "height": 1.4876600503921509,
        },
        "vehicle.toyota.prius": {
            "width": 2.006814479827881,
            "length": 4.513522624969482,
            "height": 1.5248334407806396,
        },
        "vehicle.vespa.zx125": {
            "width": 0.8659406304359436,
            "length": 1.8171066045761108,
            "height": 1.5900908708572388,
        },
        "vehicle.volkswagen.t2": {
            "width": 2.069315195083618,
            "length": 4.4804368019104,
            "height": 2.0377919673919678,
        },
        "vehicle.volkswagen.t2_2021": {
            "width": 1.77456533908844,
            "length": 4.442183971405029,
            "height": 1.9872066974639893,
        },
        "vehicle.yamaha.yzf": {
            "width": 0.8659172654151917,
            "length": 2.1907684803009033,
            "height": 1.530381441116333,
        },
        "walker.pedestrian.0001": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0002": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0003": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0004": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0005": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0006": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0007": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0008": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0009": {
            "width": 0.5,
            "length": 0.5,
            "height": 1.100000023841858,
        },
        "walker.pedestrian.0010": {
            "width": 0.5,
            "length": 0.5,
            "height": 1.100000023841858,
        },
        "walker.pedestrian.0011": {
            "width": 0.5,
            "length": 0.5,
            "height": 1.100000023841858,
        },
        "walker.pedestrian.0012": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.2999999523162842,
        },
        "walker.pedestrian.0013": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.2999999523162842,
        },
        "walker.pedestrian.0014": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.2999999523162842,
        },
        "walker.pedestrian.0015": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0016": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0017": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0018": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0019": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0020": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0021": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0022": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0023": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0024": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0025": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0026": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0027": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0028": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0029": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0030": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0031": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0032": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0033": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0034": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0035": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0036": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0037": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0038": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0039": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0040": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0041": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0042": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0043": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0044": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0045": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0046": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0047": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0048": {
            "width": 0.5,
            "length": 0.5,
            "height": 1.100000023841858,
        },
        "walker.pedestrian.0049": {
            "width": 0.5,
            "length": 0.5,
            "height": 1.100000023841858,
        },
        "walker.pedestrian.0050": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0051": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
    },
    "0.9.14": {
        "static.prop.advertisement": {
            "width": 0.28684958815574646,
            "length": 1.549233317375183,
            "height": 2.442812442779541,
        },
        "static.prop.atm": {
            "width": 0.8738368153572083,
            "length": 0.7131599187850952,
            "height": 2.2757811546325684,
        },
        "static.prop.barbeque": {
            "width": 0.49884626269340515,
            "length": 0.9456468820571899,
            "height": 1.264062523841858,
        },
        "static.prop.barrel": {
            "width": 0.47143638134002686,
            "length": 0.4596165418624878,
            "height": 0.8067187070846558,
        },
        "static.prop.bench01": {
            "width": 0.735850989818573,
            "length": 1.5440508127212524,
            "height": 0.9697656035423279,
        },
        "static.prop.bench02": {
            "width": 0.5445890426635742,
            "length": 1.5636827945709229,
            "height": 0.5090624690055847,
        },
        "static.prop.bench03": {
            "width": 0.5126523971557617,
            "length": 1.58349609375,
            "height": 0.5126562118530273,
        },
        "static.prop.bike helmet": {
            "width": 0.2770470678806305,
            "length": 0.18554961681365967,
            "height": 0.1713281273841858,
        },
        "static.prop.bin": {
            "width": 0.6381588578224182,
            "length": 0.548203706741333,
            "height": 1.0591405630111694,
        },
        "static.prop.box01": {
            "width": 0.6521967053413391,
            "length": 0.6521967649459839,
            "height": 0.6915624737739563,
        },
        "static.prop.box02": {
            "width": 0.648569643497467,
            "length": 0.648569643497467,
            "height": 0.6485937237739563,
        },
        "static.prop.box03": {
            "width": 0.6700571775436401,
            "length": 0.6608889102935791,
            "height": 0.6485937237739563,
        },
        "static.prop.briefcase": {
            "width": 0.18117640912532806,
            "length": 0.543353796005249,
            "height": 0.43281248211860657,
        },
        "static.prop.brokentile01": {
            "width": 0.1760428547859192,
            "length": 0.13444174826145172,
            "height": 0.006562499795109034,
        },
        "static.prop.brokentile02": {
            "width": 0.15229617059230804,
            "length": 0.20967696607112885,
            "height": 0.007578124757856131,
        },
        "static.prop.brokentile03": {
            "width": 0.1449233442544937,
            "length": 0.15445251762866974,
            "height": 0.008515625260770321,
        },
        "static.prop.brokentile04": {
            "width": 0.12647496163845062,
            "length": 0.12972931563854218,
            "height": 0.01015624962747097,
        },
        "static.prop.busstop": {
            "width": 1.893609881401062,
            "length": 3.8760783672332764,
            "height": 2.738906145095825,
        },
        "static.prop.busstoplb": {
            "width": 3.8760783672332764,
            "length": 1.893609881401062,
            "height": 2.738906145095825,
        },
        "static.prop.calibrator": {
            "width": 0.690200924873352,
            "length": 1.5475953817367554,
            "height": 0.20195311307907104,
        },
        "static.prop.chainbarrier": {
            "width": 0.23859326541423798,
            "length": 1.4941967725753784,
            "height": 0.8887499570846558,
        },
        "static.prop.chainbarrierend": {
            "width": 0.23859326541423798,
            "length": 0.23859328031539917,
            "height": 0.8887499570846558,
        },
        "static.prop.clothcontainer": {
            "width": 1.8865405321121216,
            "length": 1.2394330501556396,
            "height": 1.810156226158142,
        },
        "static.prop.clothesline": {
            "width": 1.1180102825164795,
            "length": 6.652331352233887,
            "height": 2.0610156059265137,
        },
        "static.prop.colacan": {
            "width": 0.07167824357748032,
            "length": 0.06817007064819336,
            "height": 0.10953124612569809,
        },
        "static.prop.constructioncone": {
            "width": 0.3440696597099304,
            "length": 0.3440696597099304,
            "height": 0.5857812166213989,
        },
        "static.prop.container": {
            "width": 1.0124343633651733,
            "length": 1.9318325519561768,
            "height": 1.7142187356948853,
        },
        "static.prop.creasedbox01": {
            "width": 0.7129369974136353,
            "length": 0.8087886571884155,
            "height": 0.11781249940395355,
        },
        "static.prop.creasedbox02": {
            "width": 0.7129369974136353,
            "length": 1.6332061290740967,
            "height": 0.2116406261920929,
        },
        "static.prop.creasedbox03": {
            "width": 0.7129369974136353,
            "length": 1.6687225103378296,
            "height": 0.021718749776482582,
        },
        "static.prop.dirtdebris01": {
            "width": 1.5139321088790894,
            "length": 1.8574368953704834,
            "height": 0.1489843726158142,
        },
        "static.prop.dirtdebris02": {
            "width": 1.5139321088790894,
            "length": 1.8574368953704834,
            "height": 0.1489843726158142,
        },
        "static.prop.dirtdebris03": {
            "width": 1.5139321088790894,
            "length": 1.8574368953704834,
            "height": 0.1883593648672104,
        },
        "static.prop.doghouse": {
            "width": 1.2721054553985596,
            "length": 1.0791168212890625,
            "height": 1.0936717987060547,
        },
        "static.prop.foodcart": {
            "width": 4.638397693634033,
            "length": 2.2846763134002686,
            "height": 3.573124885559082,
        },
        "static.prop.fountain": {
            "width": 8.401816368103027,
            "length": 8.401811599731445,
            "height": 6.924375057220459,
        },
        "static.prop.garbage01": {
            "width": 0.05342773348093033,
            "length": 0.05342777073383331,
            "height": 0.08679687231779099,
        },
        "static.prop.garbage02": {
            "width": 0.05342775210738182,
            "length": 0.05342775210738182,
            "height": 0.009218749590218067,
        },
        "static.prop.garbage03": {
            "width": 0.05944278463721275,
            "length": 0.05965827777981758,
            "height": 0.07929687201976776,
        },
        "static.prop.garbage04": {
            "width": 0.055681075900793076,
            "length": 0.05727477744221687,
            "height": 0.06882812082767487,
        },
        "static.prop.garbage05": {
            "width": 0.23253268003463745,
            "length": 0.23187801241874695,
            "height": 0.05757812410593033,
        },
        "static.prop.garbage06": {
            "width": 0.21333114802837372,
            "length": 0.3661772906780243,
            "height": 0.07679687440395355,
        },
        "static.prop.gardenlamp": {
            "width": 0.28122183680534363,
            "length": 0.2972412407398224,
            "height": 0.6478124856948853,
        },
        "static.prop.glasscontainer": {
            "width": 2.0087928771972656,
            "length": 1.9547909498214722,
            "height": 1.7884374856948853,
        },
        "static.prop.gnome": {
            "width": 0.37559059262275696,
            "length": 0.36661627888679504,
            "height": 0.8829687237739563,
        },
        "static.prop.guitarcase": {
            "width": 0.1251685619354248,
            "length": 1.242393970489502,
            "height": 0.494453102350235,
        },
        "static.prop.haybale": {
            "width": 1.1933469772338867,
            "length": 1.247710943222046,
            "height": 1.1949999332427979,
        },
        "static.prop.haybalelb": {
            "width": 1.1933469772338867,
            "length": 1.247710943222046,
            "height": 1.1949999332427979,
        },
        "static.prop.ironplank": {
            "width": 1.1743810176849365,
            "length": 1.45187246799469,
            "height": 0.020546874031424522,
        },
        "static.prop.kiosk_01": {
            "width": 1.6540685892105103,
            "length": 1.6252474784851074,
            "height": 3.2453906536102295,
        },
        "static.prop.mailbox": {
            "width": 0.43842464685440063,
            "length": 0.526629626750946,
            "height": 1.157265543937683,
        },
        "static.prop.maptable": {
            "width": 0.6170762777328491,
            "length": 1.5732090473175049,
            "height": 1.181093692779541,
        },
        "static.prop.mobile": {
            "width": 0.007261085323989391,
            "length": 0.08048340678215027,
            "height": 0.16640624403953552,
        },
        "static.prop.motorhelmet": {
            "width": 0.2592744827270508,
            "length": 0.20041197538375854,
            "height": 0.2581250071525574,
        },
        "static.prop.pergola": {
            "width": 5.085033893585205,
            "length": 5.029393672943115,
            "height": 3.54156231880188,
        },
        "static.prop.plantpot01": {
            "width": 0.4757719337940216,
            "length": 1.08914053440094,
            "height": 0.5573437213897705,
        },
        "static.prop.plantpot02": {
            "width": 0.43072694540023804,
            "length": 1.0511938333511353,
            "height": 0.48695310950279236,
        },
        "static.prop.plantpot03": {
            "width": 0.43072694540023804,
            "length": 1.0511937141418457,
            "height": 0.48695310950279236,
        },
        "static.prop.plantpot04": {
            "width": 0.7366006970405579,
            "length": 4.967305660247803,
            "height": 0.12476561963558197,
        },
        "static.prop.plantpot05": {
            "width": 0.40022480487823486,
            "length": 0.40022480487823486,
            "height": 0.2800000011920929,
        },
        "static.prop.plantpot06": {
            "width": 1.5932812690734863,
            "length": 1.5932812690734863,
            "height": 0.8495312333106995,
        },
        "static.prop.plantpot07": {
            "width": 0.4757719337940216,
            "length": 0.4946027398109436,
            "height": 0.5573437213897705,
        },
        "static.prop.plantpot08": {
            "width": 0.5304248929023743,
            "length": 0.5304248929023743,
            "height": 0.48890623450279236,
        },
        "static.prop.plasticbag": {
            "width": 0.4025624990463257,
            "length": 0.3109833598136902,
            "height": 0.5523437261581421,
        },
        "static.prop.plasticchair": {
            "width": 0.7504488825798035,
            "length": 0.7304753661155701,
            "height": 1.271328091621399,
        },
        "static.prop.plastictable": {
            "width": 2.482203245162964,
            "length": 2.482203245162964,
            "height": 2.4797656536102295,
        },
        "static.prop.platformgarbage01": {
            "width": 1.7114051580429077,
            "length": 3.6332411766052246,
            "height": 0.33156248927116394,
        },
        "static.prop.purse": {
            "width": 0.3326959013938904,
            "length": 0.15921516716480255,
            "height": 0.5852343440055847,
        },
        "static.prop.shoppingbag": {
            "width": 0.2432928830385208,
            "length": 0.47446563839912415,
            "height": 0.6006249785423279,
        },
        "static.prop.shoppingcart": {
            "width": 1.207547903060913,
            "length": 0.6694015264511108,
            "height": 1.0792968273162842,
        },
        "static.prop.shoppingtrolley": {
            "width": 0.33110183477401733,
            "length": 0.5055894255638123,
            "height": 1.0936717987060547,
        },
        "static.prop.slide": {
            "width": 4.122486591339111,
            "length": 0.9425268173217773,
            "height": 1.9766405820846558,
        },
        "static.prop.streetbarrier": {
            "width": 0.3716789782047272,
            "length": 1.2149077653884888,
            "height": 1.0691405534744263,
        },
        "static.prop.streetfountain": {
            "width": 0.6702813506126404,
            "length": 0.2869437038898468,
            "height": 1.259374976158142,
        },
        "static.prop.streetsign": {
            "width": 0.1252390295267105,
            "length": 1.0643447637557983,
            "height": 2.154374837875366,
        },
        "static.prop.streetsign01": {
            "width": 0.3029319941997528,
            "length": 2.470391273498535,
            "height": 3.8779685497283936,
        },
        "static.prop.streetsign04": {
            "width": 0.12483911216259003,
            "length": 1.1708152294158936,
            "height": 2.772890567779541,
        },
        "static.prop.swing": {
            "width": 1.5768225193023682,
            "length": 4.105227947235107,
            "height": 2.572499990463257,
        },
        "static.prop.swingcouch": {
            "width": 2.362380266189575,
            "length": 1.357409954071045,
            "height": 2.2493748664855957,
        },
        "static.prop.table": {
            "width": 2.1518361568450928,
            "length": 2.1562821865081787,
            "height": 0.846484363079071,
        },
        "static.prop.trafficcone01": {
            "width": 0.8821874856948853,
            "length": 0.8828125,
            "height": 1.1332812309265137,
        },
        "static.prop.trafficcone02": {
            "width": 0.3945564925670624,
            "length": 0.455596923828125,
            "height": 1.1828906536102295,
        },
        "static.prop.trafficwarning": {
            "width": 2.8705859184265137,
            "length": 2.373429536819458,
            "height": 3.569531202316284,
        },
        "static.prop.trampoline": {
            "width": 4.131584644317627,
            "length": 4.131584644317627,
            "height": 2.801874876022339,
        },
        "static.prop.trashbag": {
            "width": 0.42955997586250305,
            "length": 0.3779701590538025,
            "height": 0.7021093368530273,
        },
        "static.prop.trashcan01": {
            "width": 0.5678514838218689,
            "length": 0.6352845430374146,
            "height": 0.8482031226158142,
        },
        "static.prop.trashcan02": {
            "width": 0.5135547518730164,
            "length": 0.5238340497016907,
            "height": 1.0454686880111694,
        },
        "static.prop.trashcan03": {
            "width": 0.5453212857246399,
            "length": 0.6293092370033264,
            "height": 0.93359375,
        },
        "static.prop.trashcan04": {
            "width": 0.5803966522216797,
            "length": 0.7887265086174011,
            "height": 1.0163280963897705,
        },
        "static.prop.trashcan05": {
            "width": 0.5803966522216797,
            "length": 0.7887265086174011,
            "height": 1.0163280963897705,
        },
        "static.prop.travelcase": {
            "width": 0.3276098966598511,
            "length": 0.5673347115516663,
            "height": 1.2625781297683716,
        },
        "static.prop.vendingmachine": {
            "width": 1.102043867111206,
            "length": 0.8728882670402527,
            "height": 2.108203172683716,
        },
        "static.prop.warningaccident": {
            "width": 1.055053472518921,
            "length": 1.3050163984298706,
            "height": 1.8602343797683716,
        },
        "static.prop.warningconstruction": {
            "width": 1.055053472518921,
            "length": 1.3050163984298706,
            "height": 1.8602343797683716,
        },
        "static.prop.wateringcan": {
            "width": 0.19403739273548126,
            "length": 0.07035235315561295,
            "height": 0.13484375178813934,
        },
        "vehicle.audi.a2": {
            "width": 1.788678526878357,
            "length": 3.705369472503662,
            "height": 1.549050211906433,
        },
        "vehicle.audi.etron": {
            "width": 2.0327565670013428,
            "length": 4.855708599090576,
            "height": 1.6493593454360962,
        },
        "vehicle.audi.tt": {
            "width": 1.9941171407699585,
            "length": 4.181210041046143,
            "height": 1.385296106338501,
        },
        "vehicle.bh.crossbike": {
            "width": 0.7165295481681824,
            "length": 0.0,
            "height": 0.0,
        },
        "vehicle.bmw.grandtourer": {
            "width": 2.241713285446167,
            "length": 4.611005783081055,
            "height": 1.6672759056091309,
        },
        "vehicle.carlamotors.carlacola": {
            "width": 2.6269896030426025,
            "length": 5.203838348388672,
            "height": 2.467444658279419,
        },
        "vehicle.carlamotors.firetruck": {
            "width": 2.8910882472991943,
            "length": 8.46804141998291,
            "height": 3.8274123668670654,
        },
        "vehicle.chevrolet.impala": {
            "width": 2.033202886581421,
            "length": 5.357479572296143,
            "height": 1.4106587171554565,
        },
        "vehicle.citroen.c3": {
            "width": 1.8508483171463013,
            "length": 3.987684965133667,
            "height": 1.6171095371246338,
        },
        "vehicle.diamondback.century": {"width": 0.0, "length": 0.0, "height": 0.0},
        "vehicle.dodge.charger_2020": {
            "width": 1.8816219568252563,
            "length": 5.0078253746032715,
            "height": 1.5347249507904053,
        },
        "vehicle.dodge.charger_police": {
            "width": 2.0384011268615723,
            "length": 4.974244117736816,
            "height": 1.5421181917190552,
        },
        "vehicle.dodge.charger_police_2020": {
            "width": 1.9297595024108887,
            "length": 5.237514495849609,
            "height": 1.638383150100708,
        },
        "vehicle.ford.ambulance": {
            "width": 2.3511743545532227,
            "length": 6.36564302444458,
            "height": 2.431375741958618,
        },
        "vehicle.ford.crown": {
            "width": 1.8007241487503052,
            "length": 5.365678787231445,
            "height": 1.5749659538269043,
        },
        "vehicle.ford.mustang": {
            "width": 1.894826889038086,
            "length": 4.717525005340576,
            "height": 1.300939917564392,
        },
        "vehicle.gazelle.omafiets": {"width": 0.0, "length": 0.0, "height": 0.0},
        "vehicle.harley-davidson.low_rider": {
            "width": 0.7457675933837891,
            "length": 0.0,
            "height": 0.0,
        },
        "vehicle.jeep.wrangler_rubicon": {
            "width": 1.9051965475082397,
            "length": 3.866220712661743,
            "height": 1.8779358863830566,
        },
        "vehicle.kawasaki.ninja": {
            "width": 0.6402643918991089,
            "length": 0.0,
            "height": 0.0,
        },
        "vehicle.lincoln.mkz_2017": {
            "width": 2.128324270248413,
            "length": 4.901683330535889,
            "height": 1.5107464790344238,
        },
        "vehicle.lincoln.mkz_2020": {
            "width": 1.8367133140563965,
            "length": 4.89238166809082,
            "height": 1.490277647972107,
        },
        "vehicle.mercedes.coupe": {
            "width": 2.1515462398529053,
            "length": 5.0267767906188965,
            "height": 1.6506516933441162,
        },
        "vehicle.mercedes.coupe_2020": {
            "width": 1.8118125200271606,
            "length": 4.673638820648193,
            "height": 1.441947340965271,
        },
        "vehicle.mercedes.sprinter": {
            "width": 1.9884328842163086,
            "length": 5.91519021987915,
            "height": 2.560655355453491,
        },
        "vehicle.micro.microlino": {
            "width": 1.4809197187423706,
            "length": 2.2072951793670654,
            "height": 1.3760247230529785,
        },
        "vehicle.mini.cooper_s": {
            "width": 1.970275640487671,
            "length": 3.805800199508667,
            "height": 1.4750301837921143,
        },
        "vehicle.mini.cooper_s_2021": {
            "width": 2.097072124481201,
            "length": 4.552699089050293,
            "height": 1.7671663761138916,
        },
        "vehicle.mitsubishi.fusorosa": {
            "width": 3.9441518783569336,
            "length": 10.272685050964355,
            "height": 4.252848148345947,
        },
        "vehicle.nissan.micra": {
            "width": 1.845113754272461,
            "length": 3.633375883102417,
            "height": 1.5012825727462769,
        },
        "vehicle.nissan.patrol": {
            "width": 1.9315929412841797,
            "length": 4.6045098304748535,
            "height": 1.8548461198806763,
        },
        "vehicle.nissan.patrol_2021": {
            "width": 2.1499669551849365,
            "length": 5.565828800201416,
            "height": 2.045147180557251,
        },
        "vehicle.seat.leon": {
            "width": 1.8161858320236206,
            "length": 4.1928300857543945,
            "height": 1.4738311767578125,
        },
        "vehicle.tesla.cybertruck": {
            "width": 2.3895740509033203,
            "length": 6.273553371429443,
            "height": 2.098191261291504,
        },
        "vehicle.tesla.model3": {
            "width": 2.163450002670288,
            "length": 4.791779518127441,
            "height": 1.4876600503921509,
        },
        "vehicle.toyota.prius": {
            "width": 2.006814479827881,
            "length": 4.513522624969482,
            "height": 1.5248334407806396,
        },
        "vehicle.vespa.zx125": {
            "width": 0.7517611384391785,
            "length": 0.0,
            "height": 0.0,
        },
        "vehicle.volkswagen.t2": {
            "width": 2.069315195083618,
            "length": 4.4804368019104,
            "height": 2.0377919673919678,
        },
        "vehicle.volkswagen.t2_2021": {
            "width": 1.77456533908844,
            "length": 4.442183971405029,
            "height": 1.9872066974639893,
        },
        "vehicle.yamaha.yzf": {"width": 0.652134358882904, "length": 0.0, "height": 0.0},
        "walker.pedestrian.0001": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0002": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0003": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0004": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0005": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0006": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0007": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0008": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0009": {
            "width": 0.5,
            "length": 0.5,
            "height": 1.100000023841858,
        },
        "walker.pedestrian.0010": {
            "width": 0.5,
            "length": 0.5,
            "height": 1.100000023841858,
        },
        "walker.pedestrian.0011": {
            "width": 0.5,
            "length": 0.5,
            "height": 1.100000023841858,
        },
        "walker.pedestrian.0012": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.2999999523162842,
        },
        "walker.pedestrian.0013": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.2999999523162842,
        },
        "walker.pedestrian.0014": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.2999999523162842,
        },
        "walker.pedestrian.0015": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0016": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0017": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0018": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0019": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0020": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0021": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0022": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0023": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0024": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0025": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0026": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0027": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0028": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0029": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0030": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0031": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0032": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0033": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0034": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0035": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0036": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0037": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0038": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0039": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0040": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0041": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0042": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0043": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0044": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0045": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0046": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0047": {
            "width": 0.3753577768802643,
            "length": 0.3753577768802643,
            "height": 1.8600000143051147,
        },
        "walker.pedestrian.0048": {
            "width": 0.5,
            "length": 0.5,
            "height": 1.100000023841858,
        },
        "walker.pedestrian.0049": {
            "width": 0.5,
            "length": 0.5,
            "height": 1.100000023841858,
        },
    },
}

ids = _pick(_IDS, _CARLA_VER)
dims = _pick(_DIMS, _CARLA_VER)


def any_in(category):
    """Return all blueprint IDs for a category; raise if none recorded."""
    model = category + "Models"
    models = ids.get(model)
    if models:
        return models
    raise InvalidScenarioError(
        f"Scenic has no '{category}' blueprints recorded for CARLA {_CARLA_VER}."
    )


def _get_dim(bp_id, key, default):
    """Return recorded dimension or ``default`` if missing/0.

    Note: CARLA 0.9.14 bbox returns 0 for some blueprints (see CARLA issue #5841).
    """
    val = dims.get(bp_id, {}).get(key)
    return val if val else default


@distributionFunction
def width(bp_id, default):
    """Get width for ``bp_id``; return ``default`` if unknown."""
    return _get_dim(bp_id, "width", default)


@distributionFunction
def length(bp_id, default):
    """Get length for ``bp_id``; return ``default`` if unknown."""
    return _get_dim(bp_id, "length", default)


@distributionFunction
def height(bp_id, default):
    """Get height for ``bp_id``; return ``default`` if unknown."""
    return _get_dim(bp_id, "height", default)
