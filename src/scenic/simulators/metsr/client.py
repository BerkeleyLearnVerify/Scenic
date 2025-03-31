import datetime
import json
import time

from websockets.sync.client import connect


class METSRClient:
    def __init__(self, host, port, max_connection_attempts=5, timeout=30, verbose=False):
        self.host = host
        self.port = port
        self.uri = f"ws://{host}:{port}"

        self.current_tick = None
        self.timeout = timeout
        self.verbose = verbose
        self._messagesLog = []

        # Establish connection
        failed_attempts = 0
        while True:
            try:
                self.ws = connect(self.uri)
                break
            except ConnectionRefusedError:
                print(
                    f"Attempt to connect to {self.uri} failed. "
                    f"Waiting for 10 seconds before trying again... "
                    f"({max_connection_attempts - failed_attempts} attempts remaining)"
                )
                failed_attempts += 1
                if failed_attempts >= max_connection_attempts:
                    raise RuntimeError("Could not connect to METS-R Sim")
                time.sleep(10)

        assert self.ws

        # Ensure server is initialized by waiting to receive an initial packet
        # (could be ANS_ready or a heartbeat)
        self.receive_msg(ignore_heartbeats=False)

    def send_msg(self, msg):
        if self.verbose:
            self._logMessage("SENT", msg)

        self.ws.send(json.dumps(msg))

    def receive_msg(self, ignore_heartbeats):
        while True:
            raw_msg = self.ws.recv(timeout=self.timeout)

            # Decode the json string
            msg = json.loads(str(raw_msg))

            if self.verbose:
                self._logMessage("RCVD", msg)

            # Every decoded msg must have a MSG_TYPE field
            assert "TYPE" in msg.keys(), "No type field in received message"
            assert msg["TYPE"].split("_")[0] in {
                "STEP",
                "ANS",
                "CTRL",
                "ATK",
            }, "Uknown message type: " + str(msg["TYPE"])

            # Ignore certain message types entirely
            if msg["TYPE"] in {"ANS_ready"}:
                continue

            # Return decoded message, if it's not an ignored heartbeat
            if not ignore_heartbeats or msg["TYPE"] != "STEP":
                return msg

    def send_receive_msg(self, msg, ignore_heartbeats):
        self.send_msg(msg)
        return self.receive_msg(ignore_heartbeats=ignore_heartbeats)

    def tick(self):
        assert (
            self.current_tick is not None
        ), "self.current_tick is None. Reset should be called first"
        msg = {"TYPE": "STEP", "TICK": self.current_tick}
        self.send_msg(msg)

        while True:
            # Move through messages until we get to an up to date heartbeat
            res = self.receive_msg(ignore_heartbeats=False)

            assert res["TYPE"] == "STEP", res["TYPE"]
            if res["TICK"] == self.current_tick + 1:
                break

        self.current_tick = res["TICK"]

    def generate_trip(self, vehID, origin=-1, destination=-1):
        msg = {"TYPE": "CTRL_generateTrip", "DATA": []}
        if not isinstance(vehID, list):
            vehID = [vehID]
        if not isinstance(origin, list):
            origin = [origin] * len(vehID)
        if not isinstance(destination, list):
            destination = [destination] * len(vehID)

        assert (
            len(vehID) == len(origin) == len(destination)
        ), "Length of vehID, origin, and destination must be the same"
        for vehID, origin, destination in zip(vehID, origin, destination):
            msg["DATA"].append(
                {"vehID": vehID, "vehType": True, "orig": origin, "dest": destination}
            )

        res = self.send_receive_msg(msg, ignore_heartbeats=True)

        assert res["TYPE"] == "CTRL_generateTrip", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res

    def query_vehicle(self, id=None, private_veh=False, transform_coords=False):
        msg = {"TYPE": "QUERY_vehicle"}
        if id is not None:
            msg["DATA"] = []
            if not isinstance(id, list):
                id = [id]
            if not isinstance(private_veh, list):
                private_veh = [private_veh] * len(id)
            if not isinstance(transform_coords, list):
                transform_coords = [transform_coords] * len(id)
            for veh_id, prv, tran in zip(id, private_veh, transform_coords):
                msg["DATA"].append(
                    {"vehID": veh_id, "vehType": prv, "transformCoord": tran}
                )

        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "ANS_vehicle", res["TYPE"]
        return res

    def reset(self, props_file):
        msg = {"TYPE": "CTRL_reset", "propertyFile": props_file}
        res = self.send_receive_msg(msg, ignore_heartbeats=True)

        assert res["TYPE"] == "CTRL_reset", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]

        self.current_tick = -1
        self.tick()
        assert self.current_tick == 0

    def close(self):
        if self.ws is not None:
            self.ws.close()
            self.ws = None

    def _logMessage(self, direction, msg):
        self._messagesLog.append(
            (
                datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                direction,
                tuple(msg.items()),
            )
        )
