"""Simulator interface for METS-R Sim."""

import math
import time

from scenic.core.simulators import Simulation, Simulator
from scenic.core.vectors import Orientation, Vector
from scenic.simulators.metsr.client import METSRClient


class METSRSimulator(Simulator):
    def __init__(self, host, port, map_name):
        super().__init__()

        self.client = METSRClient(host=host, port=port, index=42, verbose=True)
        self.client.start()
        self.client.tick()
        self.client.ready = True
        self.map_name = map_name

    def createSimulation(self, scene, **kwargs):
        return METSRSimulation(scene, self.client, self.map_name, **kwargs)

    def destroy(self):
        self.client.terminate()
        super().destroy()


class METSRSimulation(Simulation):
    def __init__(self, scene, client, map_name, **kwargs):
        self.client = client
        self.map_name = map_name
        super().__init__(scene, **kwargs)

    def setup(self):
        print("SETUP")
        self.client.reset_same()
        super().setup()  # Calls createObjectInSimulator for each object

    def createObjectInSimulator(self, obj):
        assert obj.origin
        assert obj.destination

        self.client.generate_trip(0, origin=obj.origin, destination=obj.destination)

    def step(self):
        print("STEP")
        self.client.tick()

    def getProperties(self, obj, properties):
        print("GET PROPERTIES")
        raw_data = self.client.query_vehicle(0, private_veh=True, transform_coords=True)

        position = Vector(raw_data["x"], raw_data["y"], 0)
        speed = raw_data["speed"]
        bearing = math.radians(raw_data["bearing"])
        globalOrientation = Orientation.fromEuler(bearing, 0, 0)
        yaw, pitch, roll = obj.parentOrientation.localAnglesFor(globalOrientation)
        velocity = Vector(0, speed, 0).rotatedBy(yaw)
        angularSpeed = 0
        angularVelocity = Vector(0, 0, 0)

        values = dict(
            position=position,
            velocity=velocity,
            speed=speed,
            angularSpeed=angularSpeed,
            angularVelocity=angularVelocity,
            yaw=yaw,
            pitch=pitch,
            roll=roll,
        )
        return values

    def destroy(self):
        pass
