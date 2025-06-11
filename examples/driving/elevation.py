from scenic import scenarioFromFile
from scenic.domains.driving.roads import (
    ManeuverType,
    Network,
    Road,
    Lane,
    LaneSection,
    LaneGroup,
    Intersection,
    PedestrianCrossing,
    NetworkElement,
)
import scenic

scenario = scenarioFromFile(
    path="examples/driving/test.scenic",
    model="scenic.domains.driving.model",
    mode2D=False,
)

roadNet = getattr(scenario, "roadNet", None)
car_position_list = []
road_position_list = []
print("Generating scenes and collecting ego elevations...")
check = True
for _ in range(1):
    scene, _ = scenario.generate()
    ego = scene.egoObject
    if ego.z >= 1:
        check = False
        for v in ego.lane.polygon.vertices:
            print(v)
        print(ego.position)
