from scenic.simulators.crowd_sim.simulator import CrowdSimSimulator, CrowdSimSimulation
import numpy as np
simulator CrowdSimSimulator()


class Agent:
    name: "agent"
    object_type: "agent"
    radius: 1.0
    shape: CylinderShape((self.radius, self.radius, 1.0))
    yaw: 0
    goal: Vector(0, 0, 0)
    v_pref: 1


class Human(Agent):
    name: "human"
    object_type: "human"
    # radius: Range(0.3, 0.5)
    radius: np.random.rand() * 0.2 + 0.3
    v_pref: Range(5, 1.5)
    goal: Vector(-self.position[0], -self.position[1], 0)

class Robot(Agent):
    name: "robot"
    object_type: "robot"
    radius: 0.3
    collision: False
