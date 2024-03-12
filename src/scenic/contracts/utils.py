import warnings

import shapely

from scenic.core.specifiers import Specifier
from scenic.core.type_support import toVector


def lookuplinkedObject(scene, name):
    target_objects = [
        obj for obj in scene.objects if hasattr(obj, "name") and obj.name == name
    ]

    if len(target_objects) == 0:
        raise RuntimeError(f"No object in scenario with name '{name}'")

    if len(target_objects) > 1:
        raise RuntimeError(f"Multiple objects in scenario with name '{name}'")

    return target_objects[0]


def runComponentsSimulation(scenario, components, time=200):
    # Generate a scene and override component behaviors.
    scene, _ = scenario.generate()

    linkSetBehavior(scene, components)

    # Instantiate simulator and run simulation
    simulator = scenario.getSimulator()
    simulation = simulator.simulate(scene, maxSteps=time)

    # Reset components
    for component in components:
        component.reset()


def linkSetBehavior(scene, components):
    for component in components:
        component.link(scene)

        # if component.linkedObject.behavior:
        #     warnings.warn(
        #         f"Overriding behavior of {component.linkedObjectName} with component behavior."
        #     )

        component.linkedObject._override(
            [
                Specifier(
                    "ComponentBehaviorOverride",
                    {"behavior": 1},
                    {"behavior": ComponentBehavior(component)},
                )
            ]
        )


## Component behavior
class ComponentBehavior:
    def __init__(self, behavior):
        assert len(behavior.inputs_types) == 0
        self.behavior = behavior
        self._agent = None
        self._isRunning = False

    def _assignTo(self, agent):
        if self._agent and agent is self._agent._dynamicProxy:
            # Assigned again (e.g. by override) to same agent; do nothing.
            return
        self._start(agent)

    def _start(self, agent):
        self._agent = agent
        self._isRunning = True

    def _step(self):
        _, actions = self.behavior.run({})
        return tuple(actions)

    def _stop(self, reason=None):
        self._agent = None
        self._isRunning = False


# Lead distance functions
def leadDistanceInner(pos, tpos, lane, maxDistance):
    pos = lane.centerline.project(toVector(pos))
    tpos = toVector(tpos)
    if not lane.containsPoint(tpos):
        # Check if we are in the same lane as the target. If not,
        # advance to the start of the any possible successor lane.
        covered_dist = lane.centerline.length - shapely.line_locate_point(
            lane.centerline.lineString, shapely.Point(*pos)
        )
        succ_lanes = [
            m.connectingLane if m.connectingLane else m.endLane for m in lane.maneuvers
        ]
        new_pos = lane.centerline.end

        remMaxDistance = maxDistance - covered_dist
        if remMaxDistance <= 0:
            return float("inf")

        rem_dist = min(
            (
                leadDistanceInner(new_pos, tpos, new_lane, remMaxDistance)
                for new_lane in succ_lanes
            ),
            default=maxDistance,
        )
        return covered_dist + rem_dist

    # If we're in the same lane as the target, return the accumulated distance plus
    # the remaining distance to the point
    passed_dist = shapely.line_locate_point(
        lane.centerline.lineString, shapely.Point(*pos)
    )
    total_dist = shapely.line_locate_point(
        lane.centerline.lineString, shapely.Point(*tpos)
    )
    return total_dist - passed_dist


def leadDistance(source, target, network, maxDistance=250):
    # print(source.position, target.position, hash(network))

    # Find all lanes this point could be a part of and recurse on them.
    viable_lanes = [lane for lane in network.lanes if lane.containsPoint(source.position)]

    return min(
        min(
            (
                leadDistanceInner(source, target, lane, maxDistance)
                for lane in viable_lanes
            ),
            default=maxDistance,
        ),
        maxDistance,
    )
