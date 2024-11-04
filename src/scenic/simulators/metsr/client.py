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

    def generate_trip(self, vehID, origin, destination):
        msg = {
            "TYPE": "CTRL_generateTrip",
            "vehID": vehID,
            "origin": origin,
            "destination": destination,
        }

        res = self.send_receive_msg(msg, ignore_heartbeats=True)

        assert res["TYPE"] == "CTRL_generateTrip", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]

    def query_vehicle(self, vehID, private_veh=False, transform_coords=False):
        msg = {
            "TYPE": "QUERY_vehicle",
            "ID": vehID,
            "PRV": private_veh,
            "TRAN": transform_coords,
        }

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
