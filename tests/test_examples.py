import warnings
import os
from pathlib import Path

import pytest

import scenic

EXAMPLES_TABLE = [
                 # Carla
                 (Path() / "carla" / "adjacentLanes.scenic", {"mode2D": True}, set()),
                 (Path() / "carla" / "adjacentOpposingPair.scenic", {"mode2D": True}, set()),
                 (Path() / "carla" / "backgroundActivity.scenic", {"mode2D": True}, set()),
                 (Path() / "carla" / "car.scenic", {"mode2D": True}, set()),
                 (Path() / "carla" / "pedestrian.scenic", {"mode2D": True}, set()),
                 (Path() / "carla" / "trafficLights.scenic", {"mode2D": True}, set()),
                 (Path() / "carla" / "Carla_Challenge" / "carlaChallenge1.scenic", {"mode2D": True}, set()),
                 (Path() / "carla" / "Carla_Challenge" / "carlaChallenge10.scenic", {"mode2D": True}, set()),
                 (Path() / "carla" / "Carla_Challenge" / "carlaChallenge2.scenic", {"mode2D": True}, set()),
                 (Path() / "carla" / "Carla_Challenge" / "carlaChallenge3_dynamic.scenic", {"mode2D": True}, set()),
                 (Path() / "carla" / "Carla_Challenge" / "carlaChallenge3_static.scenic", {"mode2D": True}, set()),
                 (Path() / "carla" / "Carla_Challenge" / "carlaChallenge4.scenic", {"mode2D": True}, set()),
                 (Path() / "carla" / "Carla_Challenge" / "carlaChallenge5.scenic", {"mode2D": True}, set()),
                 (Path() / "carla" / "Carla_Challenge" / "carlaChallenge6.scenic", {"mode2D": True}, set()),
                 (Path() / "carla" / "Carla_Challenge" / "carlaChallenge7.scenic", {"mode2D": True}, set()),
                 (Path() / "carla" / "Carla_Challenge" / "carlaChallenge8.scenic", {"mode2D": True}, set()),
                 (Path() / "carla" / "Carla_Challenge" / "carlaChallenge9.scenic", {"mode2D": True}, set()),
                 (Path() / "carla" / "manual_control" / "carlaChallenge1.scenic", {"mode2D": True}, set()),
                 (Path() / "carla" / "manual_control" / "carlaChallenge3_dynamic.scenic", {"mode2D": True}, set()),
                 (Path() / "carla" / "manual_control" / "carlaChallenge4.scenic", {"mode2D": True}, set()),
                 (Path() / "carla" / "manual_control" / "carlaChallenge7.scenic", {"mode2D": True}, set()),
                 (Path() / "carla" / "NHTSA_Scenarios" / "bypassing" / "bypassing_01.scenic", {"mode2D": True}, {"verifai"}),
                 (Path() / "carla" / "NHTSA_Scenarios" / "bypassing" / "bypassing_02.scenic", {"mode2D": True}, {"verifai"}),
                 (Path() / "carla" / "NHTSA_Scenarios" / "bypassing" / "bypassing_03.scenic", {"mode2D": True}, {"verifai"}),
                 (Path() / "carla" / "NHTSA_Scenarios" / "bypassing" / "bypassing_04.scenic", {"mode2D": True}, {"verifai"}),
                 (Path() / "carla" / "NHTSA_Scenarios" / "bypassing" / "bypassing_05.scenic", {"mode2D": True}, {"verifai"}),
                 (Path() / "carla" / "NHTSA_Scenarios" / "intersection" / "intersection_01.scenic", {"mode2D": True}, {"verifai"}),
                 (Path() / "carla" / "NHTSA_Scenarios" / "intersection" / "intersection_02.scenic", {"mode2D": True}, {"verifai"}),
                 (Path() / "carla" / "NHTSA_Scenarios" / "intersection" / "intersection_03.scenic", {"mode2D": True}, {"verifai"}),
                 (Path() / "carla" / "NHTSA_Scenarios" / "intersection" / "intersection_04.scenic", {"mode2D": True}, {"verifai"}),
                 (Path() / "carla" / "NHTSA_Scenarios" / "intersection" / "intersection_05.scenic", {"mode2D": True}, {"verifai"}),
                 (Path() / "carla" / "NHTSA_Scenarios" / "intersection" / "intersection_06.scenic", {"mode2D": True}, {"verifai"}),
                 (Path() / "carla" / "NHTSA_Scenarios" / "intersection" / "intersection_07.scenic", {"mode2D": True}, {"verifai"}),
                 (Path() / "carla" / "NHTSA_Scenarios" / "intersection" / "intersection_08.scenic", {"mode2D": True}, {"verifai"}),
                 (Path() / "carla" / "NHTSA_Scenarios" / "intersection" / "intersection_09.scenic", {"mode2D": True}, {"verifai"}),
                 (Path() / "carla" / "NHTSA_Scenarios" / "intersection" / "intersection_10.scenic", {"mode2D": True}, {"verifai"}),
                 (Path() / "carla" / "NHTSA_Scenarios" / "pedestrian" / "pedestrian_01.scenic", {"mode2D": True}, {"verifai"}),
                 (Path() / "carla" / "NHTSA_Scenarios" / "pedestrian" / "pedestrian_02.scenic", {"mode2D": True}, {"verifai"}),
                 (Path() / "carla" / "NHTSA_Scenarios" / "pedestrian" / "pedestrian_03.scenic", {"mode2D": True}, {"verifai"}),
                 (Path() / "carla" / "NHTSA_Scenarios" / "pedestrian" / "pedestrian_04.scenic", {"mode2D": True}, {"verifai"}),
                 (Path() / "carla" / "NHTSA_Scenarios" / "pedestrian" / "pedestrian_05.scenic", {"mode2D": True}, {"verifai"}),
                 (Path() / "carla" / "OAS_Scenarios" / "oas_scenario_05.scenic", {"mode2D": True}, {"verifai"}),
                 (Path() / "carla" / "OAS_Scenarios" / "oas_scenario_06.scenic", {"mode2D": True}, {"verifai"}),
                 # Driving
                 (Path() / "driving" / "badlyParkedCarPullingIn.scenic", {"mode2D": True}, set()),
                 (Path() / "driving" / "car.scenic", {"mode2D": True}, set()),
                 (Path() / "driving" / "pedestrian.scenic", {"mode2D": True}, set()),
                 (Path() / "driving" / "pedestrianAcrossRoad.scenic", {"mode2D": True}, set()),
                 (Path() / "driving" / "Carla_Challenge" / "carlaChallenge2.scenic", {"mode2D": True}, set()),
                 (Path() / "driving" / "Carla_Challenge" / "carlaChallenge3.scenic", {"mode2D": True}, set()),
                 (Path() / "driving" / "OAS_Scenarios" / "oas_scenario_03.scenic", {"mode2D": True}, set()),
                 (Path() / "driving" / "OAS_Scenarios" / "oas_scenario_04.scenic", {"mode2D": True}, set()),
                 (Path() / "driving" / "OAS_Scenarios" / "oas_scenario_28.scenic", {"mode2D": True}, set()),
                 (Path() / "driving" / "OAS_Scenarios" / "oas_scenario_29.scenic", {"mode2D": True}, set()),
                 (Path() / "driving" / "OAS_Scenarios" / "oas_scenario_30.scenic", {"mode2D": True}, set()),
                 (Path() / "driving" / "OAS_Scenarios" / "oas_scenario_32.scenic", {"mode2D": True}, set()),
                 # GTA
                 (Path() / "gta" / "adjacentOpposingPair.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "angledCarInFront.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "angledPlatoonInFront.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "badlyParkedCar.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "badlyParkedCar2.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "bumperToBumper.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "carInFront.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "carPairInFront.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "concrete.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "fourCars.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "mediumTraffic.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "oncomingCarInFront.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "oncomingPairInFront.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "oneCar.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "opposingPair.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "overlappingCars.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "parkedBus.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "parkedCar.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "parkedCars.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "parkedPlatoon.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "platoon.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "platoonDaytime.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "simplest.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "threeCars.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "trafficJam.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "turningPlatoon.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "twoAngledCars.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "twoCars.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "zeroCars.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "experiment1" / "fourCars-bad.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "experiment1" / "fourCars-good.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "experiment1" / "oneCar-bad.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "experiment1" / "oneCar-good.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "experiment1" / "threeCars-bad.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "experiment1" / "threeCars-good.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "experiment1" / "twoCars-bad.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "experiment1" / "twoCars-good.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "experiment3" / "1g24-close+fixedAngle.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "experiment3" / "1g24-close+model.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "experiment3" / "1g24-close.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "experiment3" / "1g24-concrete.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "experiment3" / "1g24-model+color.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "experiment3" / "1g24-noise+model+color.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "experiment3" / "1g24-noise+model.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "experiment3" / "1g24-noise.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "experiment3" / "1g24-position+angle.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "experiment3" / "1g24-position.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "experiment3" / "1g24-relative+model+color.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "experiment3" / "1g24-relative+model.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "experiment3" / "1g24-relative+noise+model+color.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "experiment3" / "1g24-relative+noise+model.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "experiment3" / "1g24-relative+noise.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "experiment3" / "1g24-relative.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "experiment3" / "oneCar-bad-close.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "experiment3" / "oneCar-close-angled.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "experiment3" / "oneCar-close.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "experiment3" / "oneCar-good-close.scenic", {"mode2D": True}, set()),
                 (Path() / "gta" / "experiment3" / "oneCar-good-dominator.scenic", {"mode2D": True}, set()),
                 # LGSVL
                 (Path() / "lgsvl" / "carla_challenge_10.scenic", {"mode2D": True}, set()),
                 (Path() / "lgsvl" / "carla_challenge_4.scenic", {"mode2D": True}, set()),
                 (Path() / "lgsvl" / "carla_challenge_7.scenic", {"mode2D": True}, set()),
                 (Path() / "lgsvl" / "oas_28.scenic", {"mode2D": True}, set()),
                 (Path() / "lgsvl" / "oas_29.scenic", {"mode2D": True}, set()),
                 (Path() / "lgsvl" / "oas_30.scenic", {"mode2D": True}, set()),
                 (Path() / "lgsvl" / "oas_31.scenic", {"mode2D": True}, set()),
                 (Path() / "lgsvl" / "oas_32.scenic", {"mode2D": True}, set()),
                 (Path() / "lgsvl" / "oas_33.scenic", {"mode2D": True}, set()),
                 (Path() / "lgsvl" / "oas_34.scenic", {"mode2D": True}, set()),
                 (Path() / "lgsvl" / "oas_scenario5.scenic", {"mode2D": True}, set()),
                 (Path() / "lgsvl" / "scenic-cut-in-new.scenic", {"mode2D": True}, set()),
                 (Path() / "lgsvl" / "uberCrash.scenic", {"mode2D": True}, set()),
                 # Visualizer
                 (Path() / "visualizer" / "CAV23Figures" / "facing.scenic", {}, set()),
                 (Path() / "visualizer" / "CAV23Figures" / "on.scenic", {}, set()),
                 (Path() / "visualizer" / "CAV23Figures" / "on_modifying.scenic", {}, set()),
                 # Webots
                 (Path() / "webots" / "city_intersection" / "city_intersection.scenic", {}, set()),
                 (Path() / "webots" / "generic" / "adhoc_advanced.scenic", {}, set()),
                 (Path() / "webots" / "generic" / "adhoc_simple.scenic", {}, set()),
                 (Path() / "webots" / "mars" / "narrow.scenic", {}, set()),
                 (Path() / "webots" / "mars" / "narrowGoal.scenic", {}, set()),
                 (Path() / "webots" / "mars" / "narrowGoal_dynamic.scenic", {}, set()),
                 (Path() / "webots" / "road" / "adjacentOpposingPair.scenic", {"mode2D": True}, set()),
                 (Path() / "webots" / "road" / "badlyParkedCar2.scenic", {"mode2D": True}, set()),
                 (Path() / "webots" / "road" / "bumperToBumper.scenic", {"mode2D": True}, set()),
                 (Path() / "webots" / "road" / "car.scenic", {"mode2D": True}, set()),
                 (Path() / "webots" / "road" / "crossing.scenic", {"mode2D": True}, set()),
                 (Path() / "webots" / "road" / "lane_cones.scenic", {"mode2D": True}, set()),
                 (Path() / "webots" / "road" / "pedestrian.scenic", {"mode2D": True}, set()),
                 (Path() / "webots" / "road" / "rightAngle.scenic", {"mode2D": True}, set()),
                 (Path() / "webots" / "vacuum" / "vacuum.scenic", {}, set()),
                 (Path() / "webots" / "vacuum" / "vacuum_simple.scenic", {}, set()),
                 ]

EXEMPTED_FILES = [
                  
                 ]

def test_example_coverage():
    covered_example_files = {Path().parent / "examples" / val[0] for val in EXAMPLES_TABLE}
    exempted_example_files = {Path().parent / "examples" / val for val in EXEMPTED_FILES}

    parsed_example_files = set((Path().parent / "examples").glob("**/*.scenic"))

    for path in covered_example_files & exempted_example_files:
        assert os.path.exists(path)

    assert covered_example_files == parsed_example_files - exempted_example_files

@pytest.mark.slow
@pytest.mark.parametrize(
    "path,additional_args,deps",
    EXAMPLES_TABLE,
)
def test_examples(path, additional_args, deps):
    for dep in deps:
        pytest.importorskip(dep)

    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        true_path = Path().parent / "examples" / path
        scenario = scenic.scenarioFromFile(path=true_path, **additional_args)
        scenario.generate(maxIterations=100000)
