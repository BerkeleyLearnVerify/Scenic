"""Simulator interface for METS-R Sim."""

import datetime
import math
import time

from scenic.core.simulators import Simulation, Simulator
from scenic.core.vectors import Orientation, Vector
from scenic.simulators.metsr.client import METSRClient

_LOG_CLIENT_CALLS = True


class METSRSimulator(Simulator):
    def __init__(self, host, port, map_name, timestep=0.1):
        super().__init__()
        self.client = METSRClient(host=host, port=port, index=42, verbose=True)
        self.client.start()

        self.map_name = map_name
        self.timestep = timestep

    def createSimulation(self, scene, timestep, **kwargs):
        assert timestep is None or timestep == self.timestep
        return METSRSimulation(scene, self.client, self.map_name, self.timestep, **kwargs)

    def destroy(self):
        self.client.ws.close()
        super().destroy()


class METSRSimulation(Simulation):
    def __init__(self, scene, client, map_name, timestep, **kwargs):
        self.client = client
        self.map_name = map_name
        self.timestep = timestep

        self.next_pv_id = 0
        self.pv_id_map = {}

        self._client_calls = []

        super().__init__(scene, timestep=timestep, **kwargs)

    def setup(self):
        # Reset map
        self.client.reset_map("CARLA")

        super().setup()  # Calls createObjectInSimulator for each object

    def createObjectInSimulator(self, obj):
        assert obj.origin
        assert obj.destination

        import time

        start_time = time.time()

        call_kwargs = {
            "vehID": self.getPrivateVehId(obj),
            "origin": obj.origin,
            "destination": obj.destination,
        }

        if _LOG_CLIENT_CALLS:
            self._logClientCall("GENERATE_TRIP", tuple(call_kwargs.items()))

        success = self.client.generate_trip(**call_kwargs)
        assert success

    def step(self):
        if _LOG_CLIENT_CALLS:
            self._logClientCall("TICK", tuple())

        self.client.tick()

    def getProperties(self, obj, properties):
        call_kwargs = {
            "id": self.getPrivateVehId(obj),
            "private_veh": True,
            "transform_coords": True,
        }

        if _LOG_CLIENT_CALLS:
            self._logClientCall("QUERY_VEHICLE", tuple(call_kwargs.items()))

        success, raw_data = self.client.query_vehicle(**call_kwargs)
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
        if _LOG_CLIENT_CALLS:
            print("Client Calls:")
            print("[")
            for call in self._client_calls:
                print(f"    {call},")
            print("]")
        pass

    def getPrivateVehId(self, obj):
        if obj not in self.pv_id_map:
            self.pv_id_map[obj] = self.next_pv_id
            self.next_pv_id += 1

        return self.pv_id_map[obj]

    def _logClientCall(self, type, args):
        self._client_calls.append(
            (datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"), type, args)
        )
