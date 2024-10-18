"""Simulator interface for METS-R Sim."""

import math
import time

from scenic.core.simulators import Simulation, Simulator
from scenic.core.vectors import Orientation, Vector
from scenic.simulators.metsr.client import METSRClient


class METSRSimulator(Simulator):
    def __init__(self, host, port, map_name):
        super().__init__()
        self.host = host
        self.port = port
        self.map_name = map_name

    def createSimulation(self, scene, **kwargs):
        client = METSRClient(host=self.host, port=self.port, index=42, verbose=True)
        return METSRSimulation(scene, client, self.map_name, **kwargs)

    def destroy(self):
        super().destroy()


class METSRSimulation(Simulation):
    def __init__(self, scene, client, map_name, **kwargs):
        self.client = client
        self.map_name = map_name
        super().__init__(scene, **kwargs)

    def setup(self):
        # Initialize METS-R Sim Client and Server Connection
        self.client.start()
        self.client.reset_map("CARLA")

        super().setup()  # Calls createObjectInSimulator for each object

    def createObjectInSimulator(self, obj):
        assert obj.origin
        assert obj.destination

        success = self.client.generate_trip(
            0, origin=obj.origin, destination=obj.destination
        )
        assert success

    def step(self):
        self.client.tick()

    def getProperties(self, obj, properties):
        success, raw_data = self.client.query_vehicle(
            0, private_veh=True, transform_coords=True
        )
        assert success

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
        self.client.ws.close()
