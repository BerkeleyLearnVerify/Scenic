"""Simulator interface for METS-R Sim."""

import math

from scenic.core.simulators import Simulation, Simulator
from scenic.core.vectors import Orientation, Vector
from scenic.simulators.metsr.client import METSRClient


class METSRSimulator(Simulator):
    def __init__(self, host, port, map_name, timestep, sim_timestep, verbose=False):
        super().__init__()
        self.client = METSRClient(host=host, port=port, verbose=verbose)

        self.map_name = map_name
        self.timestep = timestep
        self.sim_timestep = sim_timestep

    def createSimulation(self, scene, timestep, **kwargs):
        assert timestep is None or timestep == self.timestep
        return METSRSimulation(
            scene, self.client, self.map_name, self.timestep, self.sim_timestep, **kwargs
        )

    def destroy(self):
        self.client.close()
        super().destroy()


class METSRSimulation(Simulation):
    def __init__(self, scene, client, map_name, timestep, sim_timestep, **kwargs):
        self.client = client
        self.map_name = map_name

        self.timestep = timestep
        self.sim_timestep = sim_timestep
        self.sim_ticks_per = int(timestep / sim_timestep)
        assert self.sim_ticks_per == timestep / sim_timestep

        self.next_pv_id = 0
        self.pv_id_map = {}

        self._client_calls = []

        self.count = 0

        super().__init__(scene, timestep=timestep, **kwargs)

    def setup(self):
        # Reset map
        self.client.reset("Data.properties.CARLA")

        super().setup()  # Calls createObjectInSimulator for each object

    def createObjectInSimulator(self, obj):
        assert obj.origin
        assert obj.destination

        call_kwargs = {
            "vehID": self.getPrivateVehId(obj),
            "origin": obj.origin,
            "destination": obj.destination,
        }

        self.client.generate_trip(**call_kwargs)

    def step(self):
        self.count += 1
        if self.count % 100 == 0:
            print(".", end="", flush=True)
        for _ in range(self.sim_ticks_per):
            self.client.tick()

    def updateObjects(self):
        obj_veh_ids = [self.getPrivateVehId(obj) for obj in self.objects]
        raw_veh_data = self.client.query_vehicle(obj_veh_ids, True, True)
        self.obj_data_cache = {obj: raw_veh_data[i] for i, obj in enumerate(self.objects)}
        super().updateObjects()
        self.obj_data_cache = None

    def getProperties(self, obj, properties):
        call_kwargs = {
            "vehID": self.getPrivateVehId(obj),
            "private_veh": True,
            "transform_coords": True,
        }

        raw_data = self.obj_data_cache[obj]

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
        if self.client.verbose:
            print("Client Messages Log:")
            print("[")
            for call in self.client._messagesLog:
                print(f"    {call},")
            print("]")

    def getPrivateVehId(self, obj):
        if obj not in self.pv_id_map:
            self.pv_id_map[obj] = self.next_pv_id
            self.next_pv_id += 1

        return self.pv_id_map[obj]
