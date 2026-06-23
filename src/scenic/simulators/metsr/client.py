import os
import threading
import time
from datetime import datetime

import networkx as nx
import json
from websockets.sync.client import connect

from .util import (
    VEHICLE_SENSOR_CV2X,
    VEHICLE_SENSOR_DSRC,
    VEHICLE_SENSOR_MOBILE_DEVICE,
    VEHICLE_SENSOR_TYPES,
    _as_list,
    _broadcast,
    _configured_trajectory_roots,
    _is_sequence,
    _latest_trajectory_directory,
    _looks_like_centerline,
    _normalize_sensor_type,
    _read_trajectory_manifest,
    _request_id_from_record,
    _request_zone_from_record,
    _resolve_trajectory_root,
    _set_road_reference,
    _trajectory_format_name,
    _trajectory_format_score,
    _trajectory_manifest_summary,
    run_visualization_server,
    stop_visualization_server,
    str_list_mapper_gen,
)

str_list_to_int_list = str_list_mapper_gen(int)
str_list_to_float_list = str_list_mapper_gen(float)

"""
Implementation of the remote data client

A client directly communicates with a specific METSR-SIM server.

Acknowledgement: Eric Vin for helping with the revision of the code
"""

# 2. listerize the query and control function by adding a for loop (is list, go for list, otherwise make it a list with one element)

class METSRClient:
    SENSOR_DSRC = VEHICLE_SENSOR_DSRC
    SENSOR_CV2X = VEHICLE_SENSOR_CV2X
    SENSOR_MOBILE_DEVICE = VEHICLE_SENSOR_MOBILE_DEVICE
    VEHICLE_SENSOR_TYPES = VEHICLE_SENSOR_TYPES

    def __init__(
            self,
            host,
            port,
            sim_folder = None,
            manager = None,
            max_connection_attempts = 5,
            timeout = 30,
            verbose = False,
            connection_retry_interval = 0.5,
            connection_open_timeout = 1,
            max_connection_wait = None):
        super().__init__()

        # Websocket config
        self.host = host
        self.port = port
        self.uri = f"ws://{host}:{port}"

        self.sim_folder = sim_folder # this is required for open the visualization server
        self.state = "connecting"
        self.timeout = timeout  # time out for resending the same message if no response
        self.verbose = verbose
        self._messagesLog = []

        # a pointer to the manager, for HPC usage that one manager controls multiple clients
        self.manager = manager

        # visualization server and event
        self.viz_server = None
        self.viz_event = None
        self.viz_port = None
 
        # Track the tick of the corresponding simulator
        self.current_tick = None

        # Establish connection
        connection_start = time.time()
        max_connection_wait = (
            max_connection_wait
            if max_connection_wait is not None
            else max_connection_attempts * 10
        )
        failed_attempts = 0
        while True:
            try:
                self.ws = connect(
                    self.uri,
                    max_size = 10 * 1024 * 1024,
                    ping_interval = None,
                    ping_timeout = None,
                    open_timeout = connection_open_timeout,
                )
                self.state = "connected"
                if self.verbose:
                    print(f"Connected to {self.uri}")
                break
            except OSError as exc:
                failed_attempts += 1
                elapsed = time.time() - connection_start
                if elapsed >= max_connection_wait:
                    self.state = "failed"
                    raise RuntimeError(
                        f"Could not connect to METS-R SIM at {self.uri} "
                        f"after {elapsed:.1f} seconds and {failed_attempts} attempts"
                    ) from exc

                sleep_seconds = min(connection_retry_interval, max_connection_wait - elapsed)
                if self.verbose:
                    print(
                        f"Attempt {failed_attempts} to connect to {self.uri} failed. "
                        f"Retrying in {sleep_seconds:.1f} seconds..."
                    )
                time.sleep(sleep_seconds)
                

        print("Connection established!")

        # Ensure server is initialized by waiting to receive an initial packet
        # (could be ANS_ready or a heartbeat)
        self.receive_msg(ignore_heartbeats=False, return_ready=True)

        self.lock = threading.Lock()

    def _fatal_log_error(self):
        if not self.sim_folder:
            return None

        log_path = os.path.join(self.sim_folder, "logs", "mets_r.log")
        try:
            with open(log_path, "rb") as log_file:
                log_file.seek(0, os.SEEK_END)
                size = log_file.tell()
                log_file.seek(max(0, size - 65536))
                tail = log_file.read().decode("utf-8", errors="replace")
        except OSError:
            return None

        if "FATAL JVM ERROR" not in tail and "Unresolved compilation problem" not in tail:
            return None

        lines = tail.splitlines()
        marker_index = 0
        for index, line in enumerate(lines):
            if "FATAL JVM ERROR" in line or "Unresolved compilation problem" in line:
                marker_index = index

        excerpt = "\n".join(lines[marker_index:marker_index + 8])
        guidance = ""
        if "maxWaitingTime cannot be resolved or is not a field" in tail:
            guidance = (
                "\n\nThis matches a METS-R_SIM build issue in the maxWaitingTime "
                "Control API change. Add an Integer maxWaitingTime field to "
                "MessageClass.ZoneIDOrigDestRouteNameNum and rebuild METS-R_SIM."
            )
        return f"METS-R simulator reported a fatal JVM error in {log_path}:\n{excerpt}{guidance}"

    def send_msg(self, msg):
        if self.verbose:
            self._logMessage("SENT", msg)
        self.ws.send(json.dumps(msg))

    def _update_current_tick_from_message(self, msg):
        msg_type = msg.get("TYPE")
        if msg_type not in {"STEP", "ANS_tick", "CTRL_load", "CTRL_reset"}:
            return False
        tick_value = msg.get("TICK", msg.get("tick"))
        if tick_value is None:
            return False
        server_tick = int(tick_value)
        if msg_type in {"CTRL_load", "CTRL_reset"}:
            self.current_tick = server_tick
            return True
        if self.current_tick is None or server_tick > int(self.current_tick):
            self.current_tick = server_tick
            return True
        return False

    def receive_msg(
            self,
            ignore_heartbeats,
            waiting_forever = True,
            return_ready = False,
            print_timeout = True,
            timeout = None):
        timeout = self.timeout if timeout is None else timeout
        start_time = time.time()
        while True:
            fatal_error = self._fatal_log_error()
            if fatal_error:
                raise RuntimeError(fatal_error)
            try:
                raw_msg = self.ws.recv(timeout = min(5, max(0.1, timeout)))

                # Decode the json string
                msg = json.loads(str(raw_msg))

                if self.verbose:
                    self._logMessage("RECEIVED", msg)

                # EVERY decoded msg must have a TYPE field
                if "TYPE" not in msg.keys():
                    raise RuntimeError("No type field in received message")
                if msg["TYPE"].split("_")[0] not in {"STEP", "ANS", "CTRL", "ATK"}:
                    raise RuntimeError("Uknown message type: " + str(msg["TYPE"]))

                self._update_current_tick_from_message(msg)

                # Allow tick()
                if msg["TYPE"] in {"ANS_ready"}:
                    self.current_tick = 0
                    if return_ready:
                        return msg
                    continue

                # Allow error message
                if msg["TYPE"] in {"ANS_error"}:
                    print(f"Error: {msg['MSG']}")
                    return None

                # Return decoded message, if it's not an ignored heartbeat
                if not ignore_heartbeats or msg["TYPE"] != "STEP":
                    return msg
            except KeyboardInterrupt:
                raise
            except TimeoutError:
                pass
            except Exception as exc:
                fatal_error = self._fatal_log_error()
                if fatal_error:
                    raise RuntimeError(fatal_error) from exc
                self.state = "failed"
                raise RuntimeError(f"Error while receiving message from METS-R SIM at {self.uri}: {exc}") from exc
            
            if time.time() - start_time > timeout and not waiting_forever:
                if print_timeout:
                    print("Timeout while waiting for message.")
                return None
            
    def send_receive_msg(self, msg, ignore_heartbeats, max_attempts=5): 
        with self.lock:
            res = None
            num_attempts = 0
            try:
                while res is None:
                    num_attempts += 1
                    self.send_msg(msg)
                    if(max_attempts > 0):
                        res = self.receive_msg(
                            ignore_heartbeats=ignore_heartbeats,
                            waiting_forever=False,
                            print_timeout=False,
                        )
                        if num_attempts >= max_attempts:
                            raise TimeoutError(
                                f"No response received for '{msg.get('TYPE', 'unknown')}' "
                                f"after {max_attempts} attempts; last STEP tick seen was {self.current_tick}"
                            )
                    else:
                        res = self.receive_msg(ignore_heartbeats=ignore_heartbeats, waiting_forever=True)
            except KeyboardInterrupt:
                print("\nKeyboardInterrupt detected. Stopping the current operation but keeping the server active.")
                return None  # Return None to indicate the operation was interrupted
            return res

    def _apply_tick_response(self, res):
        if res.get("TYPE") != "ANS_tick":
            raise RuntimeError(f"Expected ANS_tick, received {res.get('TYPE')}")
        if res.get("CODE", "OK") != "OK":
            raise RuntimeError(f"METS-R SIM rejected QUERY_tick: {res}")
        if "TICK" not in res:
            raise RuntimeError(f"METS-R SIM QUERY_tick response is missing TICK: {res}")
        self._update_current_tick_from_message(res)
        return int(res["TICK"])

    def _query_tick_locked(self, timeout = None):
        """Query server tick while self.lock is already held."""
        before_tick = self.current_tick
        self.send_msg({"TYPE": "QUERY_tick"})
        res = self.receive_msg(
            ignore_heartbeats=True,
            waiting_forever=False,
            print_timeout=False,
            timeout=min(5, max(0.1, self.timeout if timeout is None else timeout)),
        )
        if res is None:
            if before_tick != self.current_tick:
                return int(self.current_tick)
            return None
        return self._apply_tick_response(res)

    def query_tick(self):
        """Return the current simulation tick reported by METS-R SIM."""
        res = self.send_receive_msg({"TYPE": "QUERY_tick"}, ignore_heartbeats=True)
        return self._apply_tick_response(res)

    def query_tick_status(self):
        """Return server-side stepping status.

        Recent METS-R SIM versions include active-road stepping fields such as
        ``activeRoadStepping`` and ``activeRoadCount`` when that scheduler mode
        is enabled.
        """
        res = self.send_receive_msg({"TYPE": "QUERY_stepStatus"}, ignore_heartbeats=True)
        return res

    def tick(
            self,
            step_num = 1,
            wait_forever = False,
            retry_interval = None,
            max_wait_seconds = None,
            poll_timeout = 5,
            max_stalled_seconds = None):
        """Advance the simulator and wait until the requested tick is reached.

        ``wait_forever`` keeps tolerating slow steps, but it should not hide a
        dead server or a permanently stalled tick. Progress is therefore based
        on the server tick actually increasing, not just on receiving a reply.
        """
        assert self.current_tick is not None, "self.current_tick is None. Maybe there is another METS-R SIM instance unclosed."

        step_num = int(step_num)
        if step_num < 1:
            raise ValueError("step_num must be a positive integer")

        poll_timeout = min(max(0.1, float(poll_timeout)), max(0.1, float(self.timeout)))

        with self.lock:
            start_tick = int(self.current_tick)
            target_tick = start_tick + step_num
            overall_start = time.time()
            last_progress_time = overall_start
            last_send_time = overall_start

            if retry_interval is None:
                retry_interval = min(float(self.timeout), 30.0)

            if not wait_forever and max_wait_seconds is None:
                max_wait_seconds = self.timeout

            if wait_forever and max_stalled_seconds is None:
                max_stalled_seconds = max(60.0, min(float(self.timeout), 300.0))

            def send_step_request():
                nonlocal last_send_time
                remaining_steps = target_tick - int(self.current_tick)
                if remaining_steps <= 0:
                    return
                msg = {"TYPE": "STEP", "TICK": int(self.current_tick), "NUM": remaining_steps}
                self.send_msg(msg)
                last_send_time = time.time()

            send_step_request()

            while True:
                if int(self.current_tick) >= target_tick:
                    break

                if max_wait_seconds is not None and time.time() - overall_start > max_wait_seconds:
                    raise TimeoutError(
                        f"Timed out waiting for METS-R SIM to reach tick {target_tick}; "
                        f"last received tick was {self.current_tick}"
                    )

                # Always use a bounded receive here so wait_forever=True can still retry
                # the STEP request instead of blocking forever on a missed heartbeat.
                tick_before_receive = int(self.current_tick)
                res = self.receive_msg(
                    ignore_heartbeats=False,
                    waiting_forever=False,
                    print_timeout=False,
                    timeout=poll_timeout,
                )
                now = time.time()
                if int(self.current_tick) > tick_before_receive:
                    last_progress_time = now

                if int(self.current_tick) >= target_tick:
                    break

                if (
                        max_stalled_seconds is not None
                        and now - last_progress_time > max_stalled_seconds):
                    raise TimeoutError(
                        f"METS-R SIM made no tick progress for "
                        f"{now - last_progress_time:.1f} seconds while waiting for "
                        f"tick {target_tick}; last server tick was {self.current_tick}. "
                        "The simulator or WebSocket server is likely stalled."
                    )

                if res is None:
                    tick_before_query = int(self.current_tick)
                    synced_tick = self._query_tick_locked()
                    now = time.time()
                    if synced_tick is not None and int(self.current_tick) > tick_before_query:
                        last_progress_time = now

                    if int(self.current_tick) >= target_tick:
                        break

                    if not wait_forever:
                        raise TimeoutError(
                            f"Timed out waiting for METS-R SIM to reach tick {target_tick}; "
                            f"last received tick was {self.current_tick}"
                        )

                    should_retry_step = (
                        retry_interval is None
                        or int(self.current_tick) > tick_before_query
                        or now - max(last_send_time, last_progress_time) >= retry_interval
                    )
                    if should_retry_step:
                        if self.verbose:
                            print(
                                f"Still waiting for STEP tick {target_tick}; "
                                f"last received tick was {self.current_tick}. Retrying STEP request."
                            )
                        send_step_request()
                    continue

                if res["TYPE"] == "ANS_tick":
                    continue

                if res["TYPE"] != "STEP":
                    raise RuntimeError(f"Expected STEP while ticking, received {res['TYPE']}")

                if res.get("CODE") == "KO":
                    tick_before_query = int(self.current_tick)
                    synced_tick = self._query_tick_locked()
                    now = time.time()
                    if synced_tick is not None and int(self.current_tick) > tick_before_query:
                        last_progress_time = now
                    if int(self.current_tick) >= target_tick:
                        break
                    if not wait_forever:
                        raise RuntimeError(
                            f"METS-R SIM rejected STEP request for tick {target_tick}; "
                            f"server reported tick {self.current_tick}"
                        )
                    send_step_request()
                    continue

                step_tick = int(res["TICK"])
                if step_tick < int(self.current_tick):
                    continue

                if step_tick > int(self.current_tick):
                    self.current_tick = step_tick
                    last_progress_time = now

                if step_tick >= target_tick:
                    break
   
    # QUERY: inspect the state of the simulator
    # By default query public vehicles
    def query_vehicle(self, id = None, private_veh = False, transform_coords = False):
        """Query the full kinematic state of one or more vehicles.

        Without ``id`` the server returns two lists::

            {
              'public_vids':  [...],   # IDs of public vehicles (taxis + buses)
              'private_vids': [...],   # IDs of private vehicles (EV / GV)
              'TYPE': 'ANS_vehicle'
            }

        With ``id`` each matched vehicle produces::

            {
              'ID':      <int>   internal vehicle ID,
              'v_type':  <int>   vehicle class:
                                   0 = GV  (private gasoline vehicle)
                                   1 = ETAXI
                                   2 = EBUS
                                   3 = EV  (private electric vehicle),
              'state':   <int>   trip state (see Vehicle.java):
                                  -1 = NONE_OF_THE_ABOVE (not on network / idle)
                                   0 = PARKING
                                   1 = OCCUPIED_TRIP
                                   2 = INACCESSIBLE_RELOCATION_TRIP
                                   3 = BUS_TRIP
                                   4 = CHARGING_TRIP
                                   5 = CRUISING_TRIP
                                   6 = PICKUP_TRIP
                                   7 = ACCESSIBLE_RELOCATION_TRIP
                                   8 = PRIVATE_TRIP,
              'x':       <float> network-CRS x coordinate (or lon if transform_coords=True),
              'y':       <float> network-CRS y coordinate (or lat if transform_coords=True),
              'z':       <float> elevation,
              'bearing': <float> heading in degrees (0 = north, clockwise),
              'acc':     <float> current longitudinal acceleration (m/s²),
              'speed':   <float> current speed (m/s),
              'road':    <str>   SUMO road ID of the road the vehicle is on
                                 (only present when vehicle is on a road),
              'lane':    <int>   lane index on that road (present when on a lane),
              'dist':    <float> distance to the next downstream junction (m)
                                 (present when on a lane),
              'currentParkingRoad': <int> internal road ID where the vehicle is
                                     parked or has reserved parking, when set
            }

        Parameters
        ----------
        id : int | list[int] | None
            Vehicle ID(s) to query. Pass ``None`` to get all fleet IDs.
        private_veh : bool | list[bool]
            ``True`` for private vehicles (EV/GV), ``False`` for public
            (taxi/bus). Must match the length of ``id`` if both are lists.
        transform_coords : bool | list[bool]
            ``True`` to return WGS-84 lon/lat instead of network CRS.
        """
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
                msg["DATA"].append({"vehID": veh_id, "vehType": prv, "transformCoord": tran})

        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "ANS_vehicle", res["TYPE"]
        return res
 
    def query_taxi(self, id = None):
        """Query the state of one or more e-taxis.

        Without ``id`` returns a fleet-level summary::

            {'id_list': [...], 'TYPE': 'ANS_taxi'}

        With ``id`` each matched taxi produces::

            {
              'ID':       <int>   internal vehicle ID,
              'state':    <int>   operational state:
                                   0 = PARKING        – parked at a zone, waiting for a request
                                   1 = OCCUPIED_TRIP  – carrying passenger(s) to drop-off
                                   2 = INACCESSIBLE_RELOCATION_TRIP – repositioning, not assignable
                                   4 = CHARGING_TRIP  – heading to a charging station
                                   5 = CRUISING_TRIP  – cruising without a passenger
                                   6 = PICKUP_TRIP    – en-route to pick up a passenger
                                   7 = ACCESSIBLE_RELOCATION_TRIP – repositioning but still assignable
                                   -1 = NONE_OF_THE_ABOVE – not on network, idle, or in an unrecognized state
              'x':        <float> network-CRS x coordinate,
              'y':        <float> network-CRS y coordinate,
              'z':        <float> elevation,
              'origin':   <int>   current origin zone ID,
              'dest':     <int>   current destination zone ID
                                  (negative → heading to a charging station),
              'pass_num': <int>   number of passengers currently on board,
              'remainingDistance': <float> remaining active-trip distance in meters,
              'remainingDistanceMiles': <float> remaining active-trip distance in miles,
              'currentParkingRoad': <int> internal road ID where the taxi is
                                     parked or has reserved parking, when set
            }

        Parameters
        ----------
        id : int | list[int] | None
            Taxi ID(s) to query. Pass ``None`` to get the full fleet ID list.
        """
        my_msg = {"TYPE": "QUERY_taxi"}
        if id is not None:
            my_msg['DATA'] = []
            if not isinstance(id, list):
                id = [id]
            for i in id:
                my_msg['DATA'].append(i)

        res = self.send_receive_msg(my_msg, ignore_heartbeats=True)
        assert res["TYPE"] == "ANS_taxi", res["TYPE"]
        return res

    def query_available_taxis(self, zoneID = None):
        """Query taxis currently available for dispatch.

        Without ``zoneID`` returns available taxis across all zones. With a
        zone ID, returns only the available-taxi pool for that zone.

        Each entry in ``DATA`` includes the taxi ID, the pool zone ID, state,
        position, battery level, battery feasibility, and passenger count.
        """
        msg = {"TYPE": "QUERY_availableTaxis"}
        if zoneID is not None:
            msg["DATA"] = zoneID
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "ANS_availableTaxis", res["TYPE"]
        return res

    def query_almost_finished_taxis(
            self,
            distance_threshold_miles = None,
            distance_threshold_meters = None,
            zoneID = None):
        """Query occupied taxis expected to become available soon.

        The simulator filters to occupied taxis with one onboard request, no
        queued pickup requests, and remaining trip distance below the supplied
        threshold. A ``zoneID`` filter selects the request destination zone.
        """
        msg = {"TYPE": "QUERY_almostFinishedTaxis"}
        params = {}
        if distance_threshold_meters is not None:
            params["distanceThresholdMeters"] = distance_threshold_meters
        elif distance_threshold_miles is not None:
            params["distanceThresholdMiles"] = distance_threshold_miles
        if zoneID is not None:
            params["zoneID"] = zoneID
        if params:
            msg["DATA"] = params
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "ANS_almostFinishedTaxis", res["TYPE"]
        return res

    queryAlmostFinishedTaxis = query_almost_finished_taxis
    query_almostFinishedTaxis = query_almost_finished_taxis
         
    def query_bus(self, id = None):
        """Query the state of one or more electric buses.

        Without ``id`` returns::

            {'id_list': [...], 'TYPE': 'ANS_bus'}

        With ``id`` each matched bus produces::

            {
              'ID':            <int>   internal vehicle ID,
              'route':         <str>   name of the current bus route
                                       (empty string if the bus is idle),
              'stopZones':     <list>  zone IDs in the assigned stop sequence,
              'current_stop':  <int>   index of the last completed stop
                                       in the route's stop list (0-based),
              'pass_num':      <int>   number of passengers currently on board,
              'battery_state': <float> remaining battery energy (kWh)
            }

        Notes
        -----
        Bus trip states are the same integer codes as other vehicles
        (see :meth:`query_vehicle`), but buses only use:
          ``BUS_TRIP (3)``, ``CHARGING_TRIP (4)``, and
          ``CHARGING_RETURN_TRIP (9)`` during normal operation.

        Parameters
        ----------
        id : int | list[int] | None
            Bus ID(s) to query. Pass ``None`` to get the full fleet ID list.
        """
        my_msg = {"TYPE": "QUERY_bus"}
        if id is not None:
            my_msg['DATA'] = []
            if not isinstance(id, list):
                id = [id]
            for i in id:
                my_msg['DATA'].append(i)      
        res = self.send_receive_msg(my_msg, ignore_heartbeats=True)
        assert res["TYPE"] == "ANS_bus", res["TYPE"]
        return res

        
    def query_road(self, id = None):
        """Query static and real-time attributes of one or more roads.

        Without ``id`` returns the full road index::

            {'id_list': [...], 'orig_id': [...], 'TYPE': 'ANS_road'}

        With ``id`` (SUMO road IDs, i.e. ``orig_id`` strings) each matched
        road produces::

            {
              'ID':               <str>   SUMO original road ID,
              'r_type':           <int>   road type code,
              'num_veh':          <int>   current number of vehicles on the road,
              'speed_limit':      <float> posted speed limit (m/s),
              'avg_travel_time':  <float> recent mean travel time (s),
              'length':           <float> road length (m),
              'energy_consumed':  <float> cumulative energy consumed on this road (kWh),
              'down_stream_road': <list>  list of downstream road orig-IDs,
              'parking_capacity': <int>   parking capacity on this road,
              'parked_num':       <int>   current parked or reserved vehicles,
              'enteringVehicleQueue': <list[int]> vehicle IDs waiting to enter
                                         this road, useful for co-sim roads
            }

        Parameters
        ----------
        id : str | list[str] | None
            SUMO road ID(s) to query. Pass ``None`` to get the full road index.
        """
        my_msg = {"TYPE": "QUERY_road"}
        if id is not None:
            my_msg['DATA'] = []
            if not isinstance(id, list):
                id = [id]
            for i in id:
                my_msg['DATA'].append(i)
        res = self.send_receive_msg(my_msg, ignore_heartbeats=True)
        assert res["TYPE"] == "ANS_road", res["TYPE"]
        return res

    def query_entering_vehicle_queue(self, roadID = None):
        """Query vehicles waiting in one or more road entering queues.

        Co-simulation roads hold this queue until the external simulator calls
        :meth:`enter_road_from_queue`. Without ``roadID`` the server returns
        the road index. With road IDs, each record includes ``enteringVehicleIDs``
        and detailed queue entries with visible/private IDs, internal IDs,
        vehicle type, departure tick, and readiness.
        """
        msg = {"TYPE": "QUERY_enteringVehicleQueue"}
        if roadID is not None:
            msg["DATA"] = _as_list(roadID)
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "ANS_enteringVehicleQueue", res["TYPE"]
        return res

    def query_cosim_entering_vehicle_queue(self):
        """Query entering queues for every road currently marked as co-sim."""
        msg = {"TYPE": "QUERY_coSimEnteringVehicleQueue"}
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "ANS_coSimEnteringVehicleQueue", res["TYPE"]
        return res

    query_coSim_entering_vehicle_queue = query_cosim_entering_vehicle_queue
    query_cosim_enteringVehicleQueue = query_cosim_entering_vehicle_queue
    
    def query_centerline(self, id, lane_index = -1, transform_coords = False):
        """Query the geometric center-line of a road or a specific lane.

        Each matched road returns::

            {
              'ID':         <str>         SUMO road ID,
              'centerline': [[x, y, z], ...]  ordered coordinate list
            }

        Parameters
        ----------
        id : str | list[str]
            SUMO road ID(s) to query. Required; cannot be ``None``.
        lane_index : int | list[int]
            Index of the lane whose centerline to return.
            Use ``-1`` (default) to get the road's overall start/end points
            instead of a per-lane polyline.
        transform_coords : bool | list[bool]
            ``True`` to return network CRS lon/lat instead of WGS-84.
        """
        my_msg = {"TYPE": "QUERY_centerLine"}
        if id is not None:
            my_msg['DATA'] = []
            if not isinstance(id, list):
                id = [id]
            if not isinstance(lane_index, list):
                lane_index = [lane_index] * len(id)
            if not isinstance(transform_coords, list):
                transform_coords = [transform_coords] * len(id)
            for i, lane_idx, tran in zip(id, lane_index, transform_coords):
                my_msg['DATA'].append({"roadID": i, "laneIndex": lane_idx, "transformCoord": tran})
        else:
            raise ValueError("id cannot be None for query_centerLine")
        res = self.send_receive_msg(my_msg, ignore_heartbeats=True)
        assert res["TYPE"] == "ANS_centerLine", res["TYPE"]
        return res

    def query_zone(self, id = None):
        """Query demand and supply statistics for one or more traffic zones.

        Without ``id`` returns the full zone index::

            {'id_list': [...], 'TYPE': 'ANS_zone'}

        With ``id`` each matched zone produces::

            {
              'ID':          <int>   zone ID,
              'z_type':      <int>   zone type code,
              'taxi_demand': <int>   number of pending taxi requests currently
                                     waiting in this zone,
              'bus_demand':  <int>   number of pending bus requests currently
                                     waiting in this zone,
              'veh_stock':   <int>   number of available taxis parked / cruising
                                     in this zone at this tick,
              'x':           <float> centroid x coordinate (network CRS),
              'y':           <float> centroid y coordinate (network CRS),
              'z':           <float> centroid elevation,
              'leftTaxiRequests':    <int> cumulative taxi requests that
                                           abandoned after waiting too long,
              'leftTaxiPassengers':  <int> cumulative passengers in those
                                           abandoned taxi requests,
              'leftBusRequests':     <int> cumulative bus requests that
                                           abandoned after waiting too long,
              'leftBusPassengers':   <int> cumulative passengers in those
                                           abandoned bus requests
            }

        Parameters
        ----------
        id : int | list[int] | None
            Zone ID(s) to query. Pass ``None`` to get the full zone ID list.
        """
        my_msg = {"TYPE": "QUERY_zone"}
        if id is not None:
            my_msg['DATA'] = []
            if not isinstance(id, list):
                id = [id]
            for i in id:
                my_msg['DATA'].append(i)     
        res = self.send_receive_msg(my_msg, ignore_heartbeats=True)
        assert res["TYPE"] == "ANS_zone", res["TYPE"] 
        return res

    def query_pending_requests(self, zoneID = None):
        """Query pending taxi and bus requests.

        Without ``zoneID`` returns pending requests across all zones. With a
        zone ID, returns pending requests in that zone. Request records include
        ``ID`` (the request ID used by ``dispatch_taxi``), origin/destination
        zones and roads, party size, waiting-time fields, and a queue status.
        """
        msg = {"TYPE": "QUERY_pendingRequests"}
        if zoneID is not None:
            msg["DATA"] = zoneID
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "ANS_pendingRequests", res["TYPE"]
        return res

    def query_request(self, reqID):
        """Query one or more ride-hailing or bus request records by ID."""
        msg = {"TYPE": "QUERY_request", "DATA": []}
        if not isinstance(reqID, list):
            reqID = [reqID]
        for rid in reqID:
            msg["DATA"].append(rid)
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "ANS_request", res["TYPE"]
        return res

    def _infer_request_zone_id(self, reqID):
        response = self.query_request(reqID)
        for record in response.get("DATA", []):
            zone_id = _request_zone_from_record(record)
            if zone_id is not None:
                return zone_id
        return None

    def query_pickup_taxi_info(self, reqID=None):
        """Query taxi requests that have been matched but not picked up.

        METS-R SIM added this endpoint with the cancellation API. Passing
        ``reqID`` filters the result to one or more request IDs.
        """
        msg = {"TYPE": "QUERY_pickupTaxiInfo"}
        if reqID is not None:
            msg["DATA"] = _as_list(reqID)
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "ANS_pickupTaxiInfo", res["TYPE"]
        return res

    queryPickupTaxiInfo = query_pickup_taxi_info
    query_pickupTaxiInfo = query_pickup_taxi_info

    def query_occupied_taxi_info(self, reqID=None):
        """Query taxi requests that are already on board a taxi.

        Passing ``reqID`` filters the result to one or more request IDs.
        Cancellation eligibility is reported by the simulator in the response
        to :meth:`cancel_requests`.
        """
        msg = {"TYPE": "QUERY_occupiedTaxiInfo"}
        if reqID is not None:
            msg["DATA"] = _as_list(reqID)
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "ANS_occupiedTaxiInfo", res["TYPE"]
        return res

    queryOccupiedTaxiInfo = query_occupied_taxi_info
    query_occupiedTaxiInfo = query_occupied_taxi_info

    def query_signal(self, id = None):
        """Query the phase state of one or more traffic signals.

        Without ``id`` returns the full signal index::

            {'id_list': [...], 'TYPE': 'ANS_signal'}

        With ``id`` each matched signal produces::

            {
              'ID':               <int>  internal signal ID,
              'groupID':          <str>  SUMO junction ID this signal belongs to,
              'state':            <int>  current phase:
                                           0 = Green
                                           1 = Yellow
                                           2 = Red,
              'nex_state':        <int>  next phase (same encoding),
              'next_update_time': <int>  simulation tick at which the phase will change,
              'phase_ticks':      <list> [green_ticks, yellow_ticks, red_ticks]
                                         duration of each phase in simulation ticks
            }

        Notes
        -----
        Phase cycle order is always Green → Yellow → Red → Green.
        Each tick corresponds to ``GlobalVariables.SIMULATION_STEP_SIZE`` seconds
        (typically 0.5 s, so multiply ticks × 0.5 to get seconds).

        Use :meth:`query_signal_group` to map a SUMO junction ID to the list of
        individual signal IDs it contains.

        Parameters
        ----------
        id : int | list[int] | None
            Signal ID(s) to query. Pass ``None`` to get the full signal ID list.
        """
        my_msg = {"TYPE": "QUERY_signal"}
        if id is not None:
            my_msg['DATA'] = []
            if not isinstance(id, list):
                id = [id]
            for i in id:
                my_msg['DATA'].append(i)
        res = self.send_receive_msg(my_msg, ignore_heartbeats=True)
        assert res["TYPE"] == "ANS_signal", res["TYPE"]
        return res
    
    def query_signal_group(self, id = None):
        """Query the set of signal IDs that belong to each signal group (junction).

        Without ``id`` returns all group IDs::

            {'id_list': [...], 'TYPE': 'ANS_signalGroup'}

        With ``id`` (SUMO junction / group IDs) each group produces::

            {
              'groupID':   <str>   SUMO junction ID,
              'signalIDs': <list>  list of individual signal IDs in this group
            }

        Parameters
        ----------
        id : str | list[str] | None
            Group / junction ID(s) to query. Pass ``None`` to list all groups.
        """
        my_msg = {"TYPE": "QUERY_signalGroup"}
        if id is not None:
            my_msg['DATA'] = []
            if not isinstance(id, list):
                id = [id]
            for i in id:
                my_msg['DATA'].append(i)
        res = self.send_receive_msg(my_msg, ignore_heartbeats=True)
        assert res["TYPE"] == "ANS_signalGroup", res["TYPE"]
        return res
    
    def query_signal_between_roads(self, upstream_road, downstream_road):
        """Query the signal controlling the connection between two consecutive roads.

        Each connection produces::

            {
              'upStreamRoad':    <str>  SUMO ID of the upstream road,
              'downStreamRoad':  <str>  SUMO ID of the downstream road,
              'signalID':        <int>  signal ID (use with :meth:`query_signal`),
              'state':           <int>  current phase (0=Green, 1=Yellow, 2=Red),
              'next_state':      <int>  next phase (same encoding),
              'next_update_tick':<int>  simulation tick at which the phase changes,
              'phase_ticks':     <list> [green_ticks, yellow_ticks, red_ticks],
              'junction_id':     <int>  internal junction ID,
              'STATUS':          <str>  'OK' or 'KO' (road / junction not found)
            }

        Parameters
        ----------
        upstream_road : str | list[str]
            SUMO road ID(s) of the upstream road.
        downstream_road : str | list[str]
            SUMO road ID(s) of the downstream road (matched pairwise).
        """
        msg = {"TYPE": "QUERY_signalForConnection", "DATA": []}
        if not isinstance(upstream_road, list):
            upstream_road = [upstream_road]
        if not isinstance(downstream_road, list):
            downstream_road = [downstream_road] * len(upstream_road)
        assert len(upstream_road) == len(downstream_road), "Length of upstream_road and downstream_road must be the same"
        
        for up_road, down_road in zip(upstream_road, downstream_road):
            msg["DATA"].append({"upStreamRoad": up_road, "downStreamRoad": down_road})
        
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "ANS_signalForConnection", res["TYPE"]
        return res
    
    def query_chargingStation(self, id = None):
        """Query the status and capacity of one or more charging stations.

        Without ``id`` returns the full station index::

            {'id_list': [...], 'TYPE': 'ANS_chargingStation'}

        With ``id`` each matched station produces::

            {
              'ID':                <int>   internal station ID,
              'l2_charger':        <int>   total number of L2 (AC) charger ports,
              'dcfc_charger':      <int>   total number of DCFC (L3 / fast) charger ports,
              'l2_price':          <float> current price per kWh at L2 chargers,
              'dcfc_price':        <float> current price per kWh at DCFC chargers,
              'bus_charger':       <int>   total number of bus-dedicated charger ports,
              'num_available_l2':  <int>   number of L2 ports not currently occupied,
              'num_available_dcfc':<int>   number of DCFC ports not currently occupied,
              'x':                 <float> station x coordinate (network CRS),
              'y':                 <float> station y coordinate (network CRS),
              'z':                 <float> elevation
            }

        Notes
        -----
        Charger type codes used internally by the server:
          ``ChargingStation.L2 = 0``  (AC Level-2),
          ``ChargingStation.L3 = 1``  (DC fast charging),
          ``ChargingStation.BUS = 2`` (depot charger for electric buses).

        Parameters
        ----------
        id : int | list[int] | None
            Charging station ID(s) to query. Pass ``None`` to get the full list.
        """
        my_msg = {"TYPE": "QUERY_chargingStation"}
        if id is not None:
            my_msg['DATA'] = []
            if not isinstance(id, list):
                id = [id]
            for i in id:
                my_msg['DATA'].append(i)      
        res = self.send_receive_msg(my_msg, ignore_heartbeats=True)
        assert res["TYPE"] == "ANS_chargingStation", res["TYPE"]
        return res
    
    def query_coSimVehicle(self):
        """Query vehicles currently on co-simulation (CARLA-managed) roads.

        Returns all vehicles that are located on roads previously registered
        with :meth:`set_cosim_road`.  Each entry in ``DATA`` represents one
        vehicle::

            {
              'ID':        <int>   vehicle ID
                                    – for private vehicles (EV/GV) this is the
                                      *external* private-vehicle ID
                                    – for public vehicles (taxi/bus) this is the
                                      internal simulation ID,
              'v_type':    <bool>  True  → private vehicle (EV / GV)
                                   False → public vehicle (taxi / bus),
              'coord_map': <list>  recent coordinate history (up to 6 entries),
                                   each entry is [x, y, z, bearing, speed],
              'route':     <list>  list of upcoming road orig-IDs in the vehicle's
                                   current planned route
            }

        Returns
        -------
        dict
            ``{'DATA': [...], 'TYPE': 'ANS_coSimVehicle'}``
        """
        my_msg = {"TYPE": "QUERY_coSimVehicle"}
        res = self.send_receive_msg(my_msg, ignore_heartbeats=True)
        assert res["TYPE"] == "ANS_coSimVehicle", res["TYPE"]
        return res
    
    def query_route(self, orig_x, orig_y, dest_x, dest_y, transform_coords = False):
        """Query the shortest path between two geographic coordinates.

        Each query pair returns::

            {'road_list': [<road_orig_id>, ...]}

        or ``'KO'`` if no path was found.

        Parameters
        ----------
        orig_x, orig_y : float | list[float]
            Origin coordinate(s) in network CRS (or lon/lat if
            ``transform_coords=True``).
        dest_x, dest_y : float | list[float]
            Destination coordinate(s) (matched pairwise with origin).
        transform_coords : bool | list[bool]
            ``True`` if the input coordinates are WGS-84 lon/lat and should be
            projected into the network CRS before routing.
        """
        msg = {"TYPE": "QUERY_routesBwCoords", "DATA": []}
        if not isinstance(orig_x, list):
            orig_x = [orig_x]
            orig_y = [orig_y]
            dest_x = [dest_x]
            dest_y = [dest_y]

        if not isinstance(transform_coords, list):
            transform_coords = [transform_coords] * len(orig_x)
        
        assert len(orig_x) == len(orig_y) == len(dest_x) == len(dest_y), "Length of orig_x, orig_y, dest_x, and dest_y must be the same"

        for orig_x, orig_y, dest_x, dest_y, transform_coord in zip(orig_x, orig_y, dest_x, dest_y, transform_coords):
            msg["DATA"].append({"origX": orig_x, "origY": orig_y, "destX": dest_x, "destY": dest_y, "transformCoord": transform_coord})

        res = self.send_receive_msg(msg, ignore_heartbeats=True)

        assert res["TYPE"] == "ANS_routesBwCoords", res["TYPE"]
        return res

    def query_k_routes(self, orig_x, orig_y, dest_x, dest_y, k, transform_coords = False):
        """Query the *k* shortest paths between two geographic coordinates.

        Each query pair returns::

            {'road_lists': [[<road_orig_id>, ...], ...]}  # k routes

        or ``'KO'`` if no path was found.

        Parameters
        ----------
        orig_x, orig_y : float | list[float]
            Origin coordinate(s).
        dest_x, dest_y : float | list[float]
            Destination coordinate(s).
        k : int | list[int]
            Number of alternative routes to return per query.
        transform_coords : bool | list[bool]
            ``True`` to interpret coordinates as WGS-84 lon/lat.
        """
        msg = {"TYPE": "QUERY_multiRoutesBwCoords", "DATA": []}
        if not isinstance(orig_x, list):
            orig_x = [orig_x]
            orig_y = [orig_y]
            dest_x = [dest_x]
            dest_y = [dest_y]
            k = [k]
        if not isinstance(transform_coords, list):
            transform_coords = [transform_coords] * len(orig_x)
        if not isinstance(k, list):
            k = [k] * len(orig_x)
        
        assert len(orig_x) == len(orig_y) == len(dest_x) == len(dest_y), "Length of orig_x, orig_y, dest_x, and dest_y must be the same"

        for orig_x, orig_y, dest_x, dest_y, transform_coord, k in zip(orig_x, orig_y, dest_x, dest_y, transform_coords, k):
            msg["DATA"].append({"origX": orig_x, "origY": orig_y, "destX": dest_x, "destY": dest_y, "transformCoord": transform_coord, "K": k})
        
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "ANS_kRoutes", res["TYPE"]
        return res
    
    def query_route_between_roads(self, orig_road, dest_road):
        """Query the shortest path between two roads identified by their SUMO IDs.

        Each query pair returns::

            {'road_list': [<road_orig_id>, ...]}

        or ``'KO'`` if no path was found.

        Parameters
        ----------
        orig_road : str | list[str]
            SUMO road ID(s) of the origin road.
        dest_road : str | list[str]
            SUMO road ID(s) of the destination road (matched pairwise).
        """
        msg = {"TYPE": "QUERY_routesBwRoads", "DATA": []}
        if not isinstance(orig_road, list):
            orig_road = [orig_road]
        
        if not isinstance(dest_road, list):
            dest_road = [dest_road] * len(orig_road)
        assert len(orig_road) == len(dest_road), "Length of orig_road and dest_road must be the same"

        for orig_road, dest_road in zip(orig_road, dest_road):
            msg["DATA"].append({"orig": orig_road, "dest": dest_road})
        
        res = self.send_receive_msg(msg, ignore_heartbeats=True)

        assert res["TYPE"] == "ANS_routesBwRoads", res["TYPE"]
        return res

    def query_k_routes_between_roads(self, orig_road, dest_road, k):
        """Query the *k* shortest paths between two roads.

        Each query pair returns::

            {'road_lists': [[<road_orig_id>, ...], ...]}  # k routes

        or ``'KO'`` if no path was found.

        Parameters
        ----------
        orig_road : str | list[str]
            SUMO road ID(s) of the origin road.
        dest_road : str | list[str]
            SUMO road ID(s) of the destination road.
        k : int | list[int]
            Number of alternative routes to return per query.
        """
        msg = {"TYPE": "QUERY_multiRoutesBwRoads", "DATA": []}
        if not isinstance(orig_road, list):
            orig_road = [orig_road]
            dest_road = [dest_road]
            k = [k]
        if not isinstance(k, list):
            k = [k] * len(orig_road)

        assert len(orig_road) == len(dest_road), "Length of orig_road and dest_road must be the same"
        
        for orig_road, dest_road, k in zip(orig_road, dest_road, k):
            msg["DATA"].append({"orig": orig_road, "dest": dest_road, "K": k})
        
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "ANS_multiRoutesBwRoads", res["TYPE"]
        return res

    def query_road_weights(self, roadID = None):
        """Query the routing-graph edge weight in seconds for one or more roads.

        Without ``roadID`` returns all road/edge IDs::

            {'id_list': [...], 'orig_id': [...], 'TYPE': 'ANS_edgeWeight'}

        With ``roadID`` each matched road produces::

            {
              'ID':              <str>   SUMO road ID,
              'r_type':          <int>   road type code,
              'avg_travel_time': <float> recent mean travel time (s),
              'length':          <float> road length (m),
              'weight':          <float> current edge weight in seconds used
                                         by the router (typically travel time,
                                         may be overridden via
                                         :meth:`update_edge_weight`)
            }

        Parameters
        ----------
        roadID : str | list[str] | None
            SUMO road ID(s). Pass ``None`` to get all edges.
        """
        msg = {"TYPE": "QUERY_edgeWeight"}
        if roadID is not None:
            msg["DATA"] = []
            if not isinstance(roadID, list):
                roadID = [roadID]
            for i in roadID:
                msg["DATA"].append(i)
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "ANS_edgeWeight", res["TYPE"]
        return res
    
    def query_bus_route(self, routeID = None):
        """Query the stop sequence and road IDs of one or more bus routes.

        Without ``routeID`` returns all route identifiers::

            {'id_list': [...], 'orig_id': [...], 'TYPE': 'ANS_busRoute'}

        With ``routeID`` (route *name* strings) each matched route produces::

            {
              'routeName': <str>   human-readable route name,
              'routeID':   <int>   internal integer route ID,
              'stopZones': <list>  ordered list of zone IDs the bus visits,
              'stopRoads': <list>  corresponding SUMO road IDs at each stop
            }

        Parameters
        ----------
        routeID : str | list[str] | None
            Route name(s) to query. Pass ``None`` to list all routes.
        """
        msg = {"TYPE": "QUERY_busRoute"}
        if routeID is not None:
            msg["DATA"] = []
            if not isinstance(routeID, list):
                routeID = [routeID]
            for i in routeID:
                msg["DATA"].append(i)
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "ANS_busRoute", res["TYPE"]
        return res
    
    def query_route_bus(self, routeID = None):
        """Query the IDs of all buses currently operating on a given route.

        Without ``routeID`` returns all route identifiers::

            {'id_list': [...], 'orig_id': [...], 'TYPE': 'ANS_busWithRoute'}

        With ``routeID`` (route *name* strings) each matched route produces::

            {
              'routeName': <str>   human-readable route name,
              'routeID':   <int>   internal integer route ID,
              'busIDs':    <list>  IDs of buses currently assigned to this route
            }

        Parameters
        ----------
        routeID : str | list[str] | None
            Route name(s) to query. Pass ``None`` to list all routes.
        """
        msg = {"TYPE": "QUERY_busWithRoute"}
        if routeID is not None:
            msg["DATA"] = []
            if not isinstance(routeID, list):
                routeID = [routeID]
            for i in routeID:
                msg["DATA"].append(i)
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "ANS_busWithRoute", res["TYPE"]
        return res

    def query_routing_graph(self):
        """Build and return the full road network as a directed NetworkX graph.

        Queries all roads and assembles a ``networkx.DiGraph`` where each node
        is a SUMO road orig-ID and each directed edge represents a downstream
        connection between consecutive roads.

        Node attributes::

            length      <float>  road length (m)
            speed_limit <float>  posted speed limit (m/s)
            r_type      <int>    road type code

        Returns
        -------
        networkx.DiGraph
            Road-level connectivity graph (no edge weights).
        """
        # Step 1: get all road IDs by querying without arguments
        all_roads_res = self.query_road()
        road_ids = all_roads_res['orig_id']

        # Step 2: query road details in batches of 10 and build the graph
        graph = nx.DiGraph()
        batch_size = 10
        for batch_start in range(0, len(road_ids), batch_size):
            batch = road_ids[batch_start : batch_start + batch_size]
            res = self.query_road(id=batch)
            for road in res['DATA']:
                src = road['ID']
                graph.add_node(src, length=road['length'], speed_limit=road['speed_limit'], r_type=road['r_type'])
                for dst in road['down_stream_road']:
                    graph.add_edge(src, dst)

        return graph

    # CONTROL: change the state of the simulator
    # generate a vehicle trip between origin and destination zones
    def generate_trip(self, vehID, origin = -1, destination = -1):
        msg = {"TYPE": "CTRL_generateTrip", "DATA": []}
        if not isinstance(vehID, list):
            vehID = [vehID]
        if not isinstance(origin, list):
            origin = [origin] * len(vehID)
        if not isinstance(destination, list):
            destination = [destination] * len(vehID)

        assert len(vehID) == len(origin) == len(destination), "Length of vehID, origin, and destination must be the same"
        for vehID, origin, destination in zip(vehID, origin, destination):
            msg["DATA"].append({"vehID": vehID, "orig": origin, "dest": destination})

        res = self.send_receive_msg(msg, ignore_heartbeats=True)

        assert res["TYPE"] == "CTRL_generateTrip", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res
    
    # generate a vehicle trip between origin and destination roads
    def generate_trip_between_roads(self, vehID, origin, destination):
        msg = {"TYPE": "CTRL_genTripBwRoads", "DATA": []}
        if not isinstance(vehID, list):
            vehID = [vehID]
        if not isinstance(origin, list):
            origin = [origin] * len(vehID)
        if not isinstance(destination, list):
            destination = [destination] * len(vehID)

        assert len(vehID) == len(origin) == len(destination), "Length of vehID, origin, and destination must be the same"
        for vehID, origin, destination in zip(vehID, origin, destination):
            msg["DATA"].append({"vehID": vehID, "orig": origin, "dest": destination})

        res = self.send_receive_msg(msg, ignore_heartbeats=True)

        assert res["TYPE"] == "CTRL_genTripBwRoads", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res


    # set the road for co-simulation
    def set_cosim_road(self, roadID):
        msg = {
                "TYPE": "CTRL_setCoSimRoad",
                "DATA": [] 
              }
        if not isinstance(roadID, list):
            roadID = [roadID]
        for i in roadID:
            msg['DATA'].append(i)
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_setCoSimRoad", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res
    
    # release the road for co-simulation
    def release_cosim_road(self, roadID):
        msg = {
                "TYPE": "CTRL_releaseCosimRoad",
                "DATA": [] 
              }
        if not isinstance(roadID, list):
            roadID = [roadID]
        for i in roadID:
            msg['DATA'].append(i)
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_releaseCosimRoad", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res

    def enter_road_from_queue(self, vehID = None, roadID = None, private_veh = None,
                              internal_vehicle_id = None, requests = None):
        """Release queued vehicle(s) onto co-simulation road(s).

        The latest METS-R SIM keeps vehicles from automatically spawning onto
        roads marked ``Road.COSIM``. Query the queue with
        :meth:`query_entering_vehicle_queue` or
        :meth:`query_cosim_entering_vehicle_queue`, then call this method when
        the external simulator is ready to spawn a queued vehicle.

        Parameters
        ----------
        vehID : int | list[int] | None
            Visible vehicle ID. For private EV/GV this is the private/external
            ID returned by the co-sim bridge; for taxis/buses it is the internal
            vehicle ID. If omitted with ``roadID``, the road queue head is
            released.
        roadID : str | list[str] | None
            Optional SUMO/original road ID. If omitted, all co-sim entering
            queues are searched for ``vehID``.
        private_veh : bool | list[bool] | None
            Optional vehicle-type filter. ``True`` means private EV/GV,
            ``False`` means public taxi/bus.
        internal_vehicle_id : int | list[int] | None
            Optional internal METS-R vehicle ID, useful when visible private IDs
            are not available.
        requests : dict | list[dict] | None
            Fully formed simulator request record(s). When provided, other
            parameters are ignored.
        """
        msg = {"TYPE": "CTRL_enterRoadFromQueue", "DATA": []}
        if requests is not None:
            msg["DATA"] = _as_list(requests)
        else:
            if vehID is None and roadID is None and internal_vehicle_id is None:
                raise ValueError("vehID, roadID, internal_vehicle_id, or requests is required")

            lengths = [
                len(value) for value in (vehID, roadID, private_veh, internal_vehicle_id)
                if _is_sequence(value) and not isinstance(value, str)
            ]
            count = max(lengths) if lengths else 1
            veh_ids = _broadcast(vehID, count)
            road_ids = _broadcast(roadID, count)
            private_flags = _broadcast(private_veh, count)
            internal_ids = _broadcast(internal_vehicle_id, count)

            for vid, rid, prv, internal_id in zip(veh_ids, road_ids, private_flags, internal_ids):
                record = {}
                if vid is not None:
                    record["vehID"] = vid
                if internal_id is not None:
                    record["internalVehicleID"] = internal_id
                if prv is not None:
                    record["vehType"] = prv
                if rid is not None:
                    record["roadID"] = rid
                msg["DATA"].append(record)

        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_enterRoadFromQueue", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res
        
    # teleport vehicle to a target location specified by road and coordiantes, only work when the road is a cosim road
    def teleport_cosim_vehicle(self, vehID, x, y, bearing, speed = 0, z = 0.0, private_veh = False, transform_coords = False):
        msg = {
                "TYPE": "CTRL_teleportCoSimVeh",
                "DATA": []
                }
        if not isinstance(vehID, list):
            vehID = [vehID]
            x = [x]
            y = [y]
            z = [z]
            speed = [speed]
            bearing = [bearing]
        if not isinstance(z, list):
            z = [z] * len(vehID)
        if not isinstance(bearing, list):
            bearing = [bearing] * len(vehID)
        if not isinstance(speed, list):
            speed = [speed] * len(vehID)
        if not isinstance(private_veh, list):
            private_veh = [private_veh] * len(vehID)
        if not isinstance(transform_coords, list):
            transform_coords = [transform_coords] * len(vehID)
        for vehID, x, y, z, bearing, speed, private_veh, transform_coords in zip(vehID, x, y, z, bearing, speed, private_veh, transform_coords):
            msg["DATA"].append({"vehID": vehID, "x": x, "y": y, "z": z, "bearing": bearing, "speed": speed, "vehType": private_veh, "transformCoord": transform_coords})
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_teleportCoSimVeh", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res
    
    # teleport vehicle to a target location specified by road/lane plus distance or projected coordinates
    def teleport_trace_replay_vehicle(
            self,
            vehID,
            roadID,
            laneID,
            dist = None,
            private_veh = False,
            x = None,
            y = None,
            transform_coords = False):
        """Teleport trace-replay vehicles by lane distance or by coordinates.

        ``dist`` is the distance to the downstream junction. Recent METS-R SIM
        versions also accept ``x``/``y`` coordinates, which are projected onto
        the target lane by the simulator; set ``transform_coords=True`` when
        those coordinates need the simulator CRS transform.
        """
        msg = {
                "TYPE": "CTRL_teleportTraceReplayVeh",
                "DATA": []
                }
        veh_ids = _as_list(vehID)
        count = len(veh_ids)

        def _field_values(value, name):
            if _is_sequence(value):
                values = list(value)
                assert len(values) == count, f"{name} must have the same length as vehID"
                return values
            return [value] * count

        road_ids = _field_values(roadID, "roadID")
        lane_ids = _field_values(laneID, "laneID")
        dists = _field_values(dist, "dist")
        private_flags = _field_values(private_veh, "private_veh")
        xs = _field_values(x, "x")
        ys = _field_values(y, "y")
        transform_flags = _field_values(transform_coords, "transform_coords")

        for veh_id, road_id, lane_id, dist_value, private_flag, x_value, y_value, transform_flag in zip(
                veh_ids, road_ids, lane_ids, dists, private_flags, xs, ys, transform_flags):
            record = {
                "vehID": veh_id,
                "roadID": road_id,
                "laneID": lane_id,
                "vehType": private_flag,
            }
            if x_value is not None or y_value is not None:
                if x_value is None or y_value is None:
                    raise ValueError("Both x and y are required for coordinate trace replay teleport")
                record["x"] = x_value
                record["y"] = y_value
                record["transformCoord"] = transform_flag
            elif dist_value is not None:
                record["dist"] = dist_value
            else:
                raise ValueError("teleport_trace_replay_vehicle requires dist or x/y")
            msg["DATA"].append(record)
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_teleportTraceReplayVeh", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res
    
    # enter the next road
    def enter_next_road(self, vehID, roadID="", private_veh = False):
        msg = {
                "TYPE": "CTRL_enterNextRoad", 
                "DATA": []
                }
        if not isinstance(vehID, list):
            vehID = [vehID]
        if not isinstance(private_veh, list):
            private_veh = [private_veh] * len(vehID)
        if not isinstance(roadID, list):
            roadID = [roadID] * len(vehID)
        
        for vehID, private_veh, roadID in zip(vehID, private_veh, roadID):
            msg["DATA"].append({"vehID": vehID, "vehType": private_veh, "roadID": roadID})

        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_enterNextRoad", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res

    # reach destination
    def reach_dest(self, vehID, private_veh = False):
        msg = {
                "TYPE": "CTRL_reachDest",
                "DATA": []
                }
        if not isinstance(vehID, list):
            vehID = [vehID]
        if not isinstance(private_veh, list):
            private_veh = [private_veh] * len(vehID)
        
        for vehID, private_veh in zip(vehID, private_veh):
            msg["DATA"].append({"vehID": vehID, "vehType": private_veh})

        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_reachDest", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res
    
    # control vehicle with specified acceleration  
    def control_vehicle(self, vehID, acc, private_veh = False):
        msg = {
                "TYPE": "CTRL_controlVeh",
                "DATA": []
                }
        if not isinstance(vehID, list):
            vehID = [vehID]
            acc = [acc]
        if not isinstance(private_veh, list):
            private_veh = [private_veh] * len(vehID)
        for vehID, acc, private_veh in zip(vehID, acc, private_veh):
            msg["DATA"].append({"vehID": vehID, "vehType": private_veh, "acc": acc})
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_controlVeh", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res
    
    def update_vehicle_sensor_type(self, vehID, sensorType, private_veh = False):
        """Update vehicle sensor type used by V2X/DSRC data collection.

        ``sensorType`` may be the simulator integer code or a readable alias:
        ``0``/``"dsrc"``, ``1``/``"cv2x"``, or
        ``2``/``"mobile_device"``. DSRC and C-V2X vehicles emit BSM-style
        safety records when ``V2X = true``; mobile-device vehicles emit link
        travel-time and energy probe records.
        """
        msg = {
                "TYPE": "CTRL_updateVehicleSensorType",
                "DATA": []
                }
        vehID = _as_list(vehID)
        if not _is_sequence(private_veh):
            private_veh = [private_veh] * len(vehID)
        else:
            private_veh = list(private_veh)
        if not _is_sequence(sensorType):
            sensorType = [sensorType] * len(vehID)
        else:
            sensorType = list(sensorType)
        assert len(vehID) == len(sensorType) == len(private_veh), \
            "vehID, sensorType, and private_veh must have the same length"
        for vehID, sensorType, private_veh in zip(vehID, sensorType, private_veh):
            msg["DATA"].append({
                "vehID": vehID,
                "sensorType": _normalize_sensor_type(sensorType),
                "vehType": private_veh,
            })
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_updateVehicleSensorType", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res

    set_vehicle_sensor_type = update_vehicle_sensor_type
    updateVehicleSensorType = update_vehicle_sensor_type
    
    # Match available taxi(s) to existing pending request(s).
    def dispatch_taxi(self, vehID, reqID):
        """Dispatch taxi(s) to serve already-pending request(s).

        The METS-R SIM Control API separates request creation from dispatching.
        Use ``add_taxi_requests`` or ``add_taxi_requests_between_roads`` first,
        read the returned ``reqID``, then pass ``vehID`` and ``reqID`` here.

        Recent METS-R SIM versions can release a parked taxi from parking,
        queue the request after an unfinished passenger-free trip, and return
        fields such as ``remainingCapacity``, ``requestPassengers``, and
        ``parkingReservationReleased`` in each response record.
        """
        msg = {
                "TYPE": "CTRL_dispatchTaxi",
                "DATA": []
                }
        if not isinstance(vehID, list):
            vehID = [vehID]
        if not isinstance(reqID, list):
            reqID = [reqID] * len(vehID)
        assert len(vehID) == len(reqID), "vehID and reqID must have the same length"

        for vehID, reqID in zip(vehID, reqID):
            msg["DATA"].append({"vehID": vehID, "reqID": reqID})
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_dispatchTaxi", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res

    def cancel_requests(self, reqID, zoneID=None):
        """Cancel one or more taxi/bus requests.

        The latest METS-R SIM control API uses ``CTRL_cancelRequests`` and
        requires the request's origin zone for each record. ``reqID`` may be a
        scalar request ID, a list of request IDs, a request record, or a list
        of request records containing request ID and origin-zone fields. When
        ``zoneID`` is omitted, the client attempts to infer it with
        :meth:`query_request`.

        The returned ``DATA`` list contains per-request ``STATUS``/``WARN``
        details from the simulator; a top-level ``CODE`` of ``OK`` only means
        the control message itself was processed.
        """
        if zoneID is None and isinstance(reqID, dict):
            request_records = [reqID]
        elif (
            zoneID is None
            and _is_sequence(reqID)
            and all(isinstance(record, dict) for record in reqID)
        ):
            request_records = list(reqID)
        else:
            request_ids = [_request_id_from_record(record) for record in _as_list(reqID)]
            if zoneID is None:
                zone_ids = [None] * len(request_ids)
            elif _is_sequence(zoneID):
                zone_ids = list(zoneID)
            else:
                zone_ids = [zoneID] * len(request_ids)
            assert len(request_ids) == len(zone_ids), \
                "reqID and zoneID must have the same length"
            request_records = [
                {"reqID": rid, "zoneID": zid}
                for rid, zid in zip(request_ids, zone_ids)
            ]

        msg = {"TYPE": "CTRL_cancelRequests", "DATA": []}
        missing_zone_ids = []
        for record in request_records:
            rid = _request_id_from_record(record)
            zid = _request_zone_from_record(record)
            if rid is None:
                raise ValueError("reqID is required for cancel_requests")
            if zid is None:
                zid = self._infer_request_zone_id(rid)
            if zid is None:
                missing_zone_ids.append(rid)
                continue
            msg["DATA"].append({"reqID": rid, "zoneID": zid})

        if missing_zone_ids:
            raise ValueError(
                "zoneID is required for cancel_requests; could not infer it "
                "for reqID(s): {}".format(missing_zone_ids)
            )

        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_cancelRequests", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res

    cancel_request = cancel_requests
    cancelRequests = cancel_requests

    def reposition_taxi(self, vehID, zoneID):
        """Reposition idle/cruising taxi(s) to destination zone(s).

        If the taxi was already traveling to reserved parking, the simulator can
        release that reservation and report ``parkingReservationReleased`` in
        the response record.
        """
        msg = {"TYPE": "CTRL_repositionTaxi", "DATA": []}
        if not isinstance(vehID, list):
            vehID = [vehID]
        if not isinstance(zoneID, list):
            zoneID = [zoneID] * len(vehID)
        assert len(vehID) == len(zoneID), "vehID and zoneID must have the same length"

        for vehID, zoneID in zip(vehID, zoneID):
            msg["DATA"].append({"vehID": vehID, "zoneID": zoneID})
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_repositionTaxi", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res

    def go_parking(self, vehID, zoneID=None, roadID=None):
        """Send idle taxi(s) to park at a target zone or road.

        Provide ``zoneID``, ``roadID``, or both. If only ``zoneID`` is supplied,
        METS-R SIM samples a parking road in that zone. If only ``roadID`` is
        supplied, the zone is inferred from the road. When both are supplied,
        the road must belong to the zone and have available parking capacity.
        """
        msg = {"TYPE": "CTRL_goParking", "DATA": []}
        vehID = _as_list(vehID)
        if zoneID is None:
            zoneID = [None] * len(vehID)
        elif not _is_sequence(zoneID):
            zoneID = [zoneID] * len(vehID)
        else:
            zoneID = list(zoneID)
        if roadID is None:
            roadID = [None] * len(vehID)
        elif not _is_sequence(roadID):
            roadID = [roadID] * len(vehID)
        else:
            roadID = list(roadID)
        assert len(vehID) == len(zoneID) == len(roadID), \
            "vehID, zoneID, and roadID must have the same length"

        for vid, zid, rid in zip(vehID, zoneID, roadID):
            if zid is None and rid is None:
                raise ValueError("zoneID or roadID is required for go_parking")
            record = {"vehID": vid}
            if zid is not None:
                record["zoneID"] = zid
            if rid is not None:
                record["roadID"] = rid
            msg["DATA"].append(record)
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_goParking", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res

    goParking = go_parking
    
    def add_taxi_requests(self, zoneID, dest, num, max_waiting_time = None, maxWaitingTime = None):
        """Add one or more pending taxi requests.

        ``max_waiting_time`` is optional and uses simulation ticks. It maps to
        the API field ``maxWaitingTime``. When it is positive, the simulator
        overrides the request's default waiting tolerance; omitted, ``None``,
        or non-positive values keep the simulator default.
        """
        if max_waiting_time is None and maxWaitingTime is not None:
            max_waiting_time = maxWaitingTime

        msg = {
                "TYPE": "CTRL_addTaxiRequests",
                "DATA": []
                }
        if not isinstance(zoneID, list):
            zoneID = [zoneID]
        if not isinstance(dest, list):
            dest = [dest] * len(zoneID)
        if not isinstance(num, list):
            num = [num] * len(zoneID)
        if max_waiting_time is None:
            max_waiting_time = [None] * len(zoneID)
        elif not isinstance(max_waiting_time, list):
            max_waiting_time = [max_waiting_time] * len(zoneID)
        assert len(zoneID) == len(dest) == len(num) == len(max_waiting_time), \
            "zoneID, dest, num, and max_waiting_time must have the same length"

        for zoneID, dest, num, max_wait in zip(zoneID, dest, num, max_waiting_time):
            record = {"zoneID": zoneID, "dest": dest, "num": num}
            if max_wait is not None:
                record["maxWaitingTime"] = max_wait
            msg["DATA"].append(record)
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_addTaxiRequests", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res
    
    def add_taxi_requests_between_roads(self, orig, dest, num):
        msg = {
                "TYPE": "CTRL_addTaxiReqBwRoads",
                "DATA": []
                }
        if not isinstance(orig, list):
            orig = [orig]
        if not isinstance(dest, list):
            dest = [dest] * len(orig)
        if not isinstance(num, list):
            num = [num] * len(orig)

        for orig, dest, num in zip(orig, dest, num):
            msg["DATA"].append({"orig": orig, "dest": dest, "num": num})
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_addTaxiReqBwRoads", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res
    
    # assign bus
    def add_bus_route(self, routeName, zone, road, paths = None):
        has_paths = paths is not None
        if not has_paths:
            msg = {
                    "TYPE": "CTRL_addBusRoute",
                    "DATA": []
                    }
        else:
            msg = {
                    "TYPE": "CTRL_addBusRouteWithPath",
                    "DATA": []
                    }
        if not isinstance(routeName, list):
            routeName = [routeName]
            zone = [zone]
            road = [road]
            if has_paths:
                paths = [paths]
        elif has_paths and not isinstance(paths, list):
            paths = [paths] * len(routeName)

        if not has_paths:
            for routeName, zone, road in zip(routeName, zone, road):
                msg["DATA"].append({"routeName": routeName, "zones": zone, "roads": road})
        else:
            for routeName, zone, road, paths in zip(routeName, zone, road, paths):
                msg["DATA"].append({"routeName": routeName, "zones": zone, "roads": road, "paths": paths})
        res = self.send_receive_msg(msg, ignore_heartbeats=True)

        if not has_paths:
            assert res["TYPE"] == "CTRL_addBusRoute", res["TYPE"]
        else:
            assert res["TYPE"] == "CTRL_addBusRouteWithPath", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res

    def add_bus_run(self, routeName, departTime):
        msg = {
                "TYPE": "CTRL_addBusRun",
                "DATA": []
                }
        if not isinstance(routeName, list):
            routeName = [routeName]
            departTime = [departTime]

        for routeName, departTime in zip(routeName, departTime):
            msg["DATA"].append({"routeName": routeName, "departTime": departTime})
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_addBusRun", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res
    
    def insert_bus_stop(self, busID, routeName, zoneID, roadName, stopIndex):
        msg = {
                "TYPE": "CTRL_insertStopToRoute",
                "DATA": []
                }
        if not isinstance(busID, list):
            busID = [busID]
            routeName = [routeName] * len(busID)
            zoneID = [zoneID] * len(busID)
            roadName = [roadName] * len(busID)
            stopIndex = [stopIndex] * len(busID)

        for busID, routeName, zoneID, roadName, stopIndex in zip(busID, routeName, zoneID, roadName, stopIndex):
            msg["DATA"].append({"busID": busID, "routeName": routeName, "zone": zoneID, "road": roadName, "stopIndex": stopIndex})

        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_insertStopToRoute", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res
    
    def remove_bus_stop(self, busID, routeName, stopIndex):
        msg = {
                "TYPE": "CTRL_removeStopFromRoute",
                "DATA": []
                }
        if not isinstance(busID, list):
            busID = [busID]
            routeName = [routeName] * len(busID)
            stopIndex = [stopIndex] * len(busID)

        for busID, routeName, stopIndex in zip(busID, routeName, stopIndex):
            msg["DATA"].append({"busID": busID, "routeName": routeName, "stopIndex": stopIndex})

        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_removeStopFromRoute", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res


    def assign_request_to_bus(self, busID, reqID):
        """Match existing pending bus request(s) to bus(es).

        The latest METS-R SIM Control API separates bus-request creation from
        assignment. Use ``add_bus_requests`` first, read the returned
        ``reqID``, then pass ``busID`` and ``reqID`` here.
        """
        msg = {
                "TYPE": "CTRL_assignRequestToBus",
                "DATA": []
                }
        if not isinstance(busID, list):
            busID = [busID]
        if not isinstance(reqID, list):
            reqID = [reqID] * len(busID)
        assert len(busID) == len(reqID), "busID and reqID must have the same length"

        for bus_id, req_id in zip(busID, reqID):
            msg["DATA"].append({"busID": bus_id, "reqID": req_id})
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_assignRequestToBus", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res
    
    def add_bus_requests(self, zoneID, dest, routeName, num, max_waiting_time = None, maxWaitingTime = None):
        """Add one or more pending bus requests.

        ``max_waiting_time`` is optional and uses simulation ticks. It maps to
        the API field ``maxWaitingTime``. When it is positive, the simulator
        overrides the request's default waiting tolerance.
        """
        if max_waiting_time is None and maxWaitingTime is not None:
            max_waiting_time = maxWaitingTime

        msg = {
                "TYPE": "CTRL_addBusRequests",
                "DATA": []
                }
        if not isinstance(zoneID, list):
            zoneID = [zoneID]
        if not isinstance(dest, list):
            dest = [dest] * len(zoneID)
        if not isinstance(num, list):
            num = [num] * len(zoneID)
        if not isinstance(routeName, list):
            routeName = [routeName] * len(zoneID)
        if max_waiting_time is None:
            max_waiting_time = [None] * len(zoneID)
        elif not isinstance(max_waiting_time, list):
            max_waiting_time = [max_waiting_time] * len(zoneID)
        assert len(zoneID) == len(dest) == len(num) == len(routeName) == len(max_waiting_time), \
            "zoneID, dest, num, routeName, and max_waiting_time must have the same length"

        for zoneID, dest, num, routeName, max_wait in zip(zoneID, dest, num, routeName, max_waiting_time):
            record = {"zoneID": zoneID, "dest": dest, "num": num, "routeName": routeName}
            if max_wait is not None:
                record["maxWaitingTime"] = max_wait
            msg["DATA"].append(record)
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_addBusRequests", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res
    
    # update vehicle route 
    def update_vehicle_route(self, vehID, route, private_veh = False):
        msg = {
                "TYPE": "CTRL_updateVehicleRoute",
                "DATA": []
                }
        if not isinstance(vehID, list):
            vehID = [vehID]
            route = [route]
        if not isinstance(private_veh, list):
            private_veh = [private_veh] * len(vehID)

        for vehID, route, private_veh in zip(vehID, route, private_veh):
            msg["DATA"].append({"vehID": vehID, "route": route, "vehType": private_veh})
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_updateVehicleRoute", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res

    # update road weights in the routing map
    def update_road_weights(self, roadID, weight):
        msg = {"TYPE": "CTRL_updateEdgeWeight", "DATA": []}
        if not isinstance(roadID, list):
            roadID = [roadID]
            weight = [weight]
        if not isinstance(weight, list):
            weight = [weight] * len(roadID)
        for roadID, weight in zip(roadID, weight):
            msg["DATA"].append({"roadID": roadID, "weight": weight})
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_updateEdgeWeight", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res

    def update_road_parking_capacity(
            self, roadID, parking_capacity=None, parkingCapacity=None, capacity=None):
        """Update parking capacity for one or more roads.

        ``parking_capacity`` is the preferred Python argument. ``parkingCapacity``
        and ``capacity`` are accepted to mirror the METS-R SIM Control API
        aliases.
        """
        if parking_capacity is None:
            parking_capacity = parkingCapacity
        if parking_capacity is None:
            parking_capacity = capacity
        if parking_capacity is None:
            raise ValueError("parking_capacity is required")

        msg = {"TYPE": "CTRL_updateRoadParkingCapacity", "DATA": []}
        roadID = _as_list(roadID)
        if not _is_sequence(parking_capacity):
            parking_capacity = [parking_capacity] * len(roadID)
        else:
            parking_capacity = list(parking_capacity)
        assert len(roadID) == len(parking_capacity), \
            "roadID and parking_capacity must have the same length"

        for rid, cap in zip(roadID, parking_capacity):
            msg["DATA"].append({"roadID": rid, "parkingCapacity": cap})
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_updateRoadParkingCapacity", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res

    updateRoadParkingCapacity = update_road_parking_capacity
    
    # update charging station prices
    def update_charging_prices(self, stationID, stationType, price):
        msg = {"TYPE": "CTRL_updateChargingPrice", "DATA": []}
        if not isinstance(stationID, list):
            stationID = [stationID]
            stationType = [stationType]
            price = [price]
        if not isinstance(stationType, list):
            stationType = [stationType] * len(stationID)
        if not isinstance(price, list):
            price = [price] * len(stationID)
        for stationID, stationType, price in zip(stationID, stationType, price):
            msg["DATA"].append({"chargerID": stationID, "chargerType": stationType, "weight": price})
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_updateChargingPrice", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res
    
    # Traffic signal phase control
    # Update the signal phase given signal ID and target phase (optionally with phase time offset)
    # If only phase is provided, starts from the beginning of that phase (phaseTime = 0)
    def update_signal(self, signalID, targetPhase, phaseTime = None):
        msg = {"TYPE": "CTRL_updateSignal", "DATA": []}
        if not isinstance(signalID, list):
            signalID = [signalID]
            targetPhase = [targetPhase]
        if not isinstance(targetPhase, list):
            targetPhase = [targetPhase] * len(signalID)
        if phaseTime is None:
            phaseTime = [None] * len(signalID)
        elif not isinstance(phaseTime, list):
            phaseTime = [phaseTime] * len(signalID)
        else:
            # If phaseTime is a list, ensure it matches the length
            if len(phaseTime) != len(signalID):
                phaseTime = phaseTime * (len(signalID) // len(phaseTime) + 1)
                phaseTime = phaseTime[:len(signalID)]
        
        assert len(signalID) == len(targetPhase) == len(phaseTime), "Length of signalID, targetPhase, and phaseTime must be the same"
        
        for sig_id, tgt_phase, ph_time in zip(signalID, targetPhase, phaseTime):
            signal_data = {"signalID": sig_id, "targetPhase": tgt_phase}
            if ph_time is not None:
                signal_data["phaseTime"] = ph_time
            msg["DATA"].append(signal_data)
        
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_updateSignal", res["TYPE"]
        return res
    
    # Update signal phase timing (green, yellow, red durations)
    def update_signal_timing(self, signalID, greenTime, yellowTime, redTime):
        msg = {"TYPE": "CTRL_updateSignalTiming", "DATA": []}
        if not isinstance(signalID, list):
            signalID = [signalID]
            greenTime = [greenTime]
            yellowTime = [yellowTime]
            redTime = [redTime]
        if not isinstance(greenTime, list):
            greenTime = [greenTime] * len(signalID)
        if not isinstance(yellowTime, list):
            yellowTime = [yellowTime] * len(signalID)
        if not isinstance(redTime, list):
            redTime = [redTime] * len(signalID)
        
        assert len(signalID) == len(greenTime) == len(yellowTime) == len(redTime), "Length of signalID, greenTime, yellowTime, and redTime must be the same"
        
        for sig_id, green, yellow, red in zip(signalID, greenTime, yellowTime, redTime):
            msg["DATA"].append({"signalID": sig_id, "greenTime": green, "yellowTime": yellow, "redTime": red})
        
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_updateSignalTiming", res["TYPE"]
        return res
    
    # Set a complete new phase plan for a signal (phase timing + starting state + offset)
    # Time values are in seconds
    def set_signal_phase_plan(self, signalID, greenTime, yellowTime, redTime, startPhase, phaseOffset = None):
        msg = {"TYPE": "CTRL_setSignalPhasePlan", "DATA": []}
        if not isinstance(signalID, list):
            signalID = [signalID]
            greenTime = [greenTime]
            yellowTime = [yellowTime]
            redTime = [redTime]
            startPhase = [startPhase]
        if not isinstance(greenTime, list):
            greenTime = [greenTime] * len(signalID)
        if not isinstance(yellowTime, list):
            yellowTime = [yellowTime] * len(signalID)
        if not isinstance(redTime, list):
            redTime = [redTime] * len(signalID)
        if not isinstance(startPhase, list):
            startPhase = [startPhase] * len(signalID)
        if phaseOffset is None:
            phaseOffset = [None] * len(signalID)
        elif not isinstance(phaseOffset, list):
            phaseOffset = [phaseOffset] * len(signalID)
        else:
            # If phaseOffset is a list, ensure it matches the length
            if len(phaseOffset) != len(signalID):
                phaseOffset = phaseOffset * (len(signalID) // len(phaseOffset) + 1)
                phaseOffset = phaseOffset[:len(signalID)]
        
        assert len(signalID) == len(greenTime) == len(yellowTime) == len(redTime) == len(startPhase) == len(phaseOffset), "Length of all parameters must match"
        
        for sig_id, green, yellow, red, start_phase, ph_offset in zip(signalID, greenTime, yellowTime, redTime, startPhase, phaseOffset):
            signal_data = {"signalID": sig_id, "greenTime": green, "yellowTime": yellow, "redTime": red, "startPhase": start_phase}
            if ph_offset is not None:
                signal_data["phaseOffset"] = ph_offset
            msg["DATA"].append(signal_data)
        
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_setSignalPhasePlan", res["TYPE"]
        return res
    
    # Set a complete new phase plan with tick-level precision
    # Time values are in simulation ticks for more precise control
    def set_signal_phase_plan_ticks(self, signalID, greenTicks, yellowTicks, redTicks, startPhase, tickOffset = None):
        msg = {"TYPE": "CTRL_setSignalPhasePlanTicks", "DATA": []}
        if not isinstance(signalID, list):
            signalID = [signalID]
            greenTicks = [greenTicks]
            yellowTicks = [yellowTicks]
            redTicks = [redTicks]
            startPhase = [startPhase]
        if not isinstance(greenTicks, list):
            greenTicks = [greenTicks] * len(signalID)
        if not isinstance(yellowTicks, list):
            yellowTicks = [yellowTicks] * len(signalID)
        if not isinstance(redTicks, list):
            redTicks = [redTicks] * len(signalID)
        if not isinstance(startPhase, list):
            startPhase = [startPhase] * len(signalID)
        if tickOffset is None:
            tickOffset = [None] * len(signalID)
        elif not isinstance(tickOffset, list):
            tickOffset = [tickOffset] * len(signalID)
        else:
            # If tickOffset is a list, ensure it matches the length
            if len(tickOffset) != len(signalID):
                tickOffset = tickOffset * (len(signalID) // len(tickOffset) + 1)
                tickOffset = tickOffset[:len(signalID)]
        
        assert len(signalID) == len(greenTicks) == len(yellowTicks) == len(redTicks) == len(startPhase) == len(tickOffset), "Length of all parameters must match"
        
        for sig_id, green, yellow, red, start_phase, tck_offset in zip(signalID, greenTicks, yellowTicks, redTicks, startPhase, tickOffset):
            signal_data = {"signalID": sig_id, "greenTicks": green, "yellowTicks": yellow, "redTicks": red, "startPhase": start_phase}
            if tck_offset is not None:
                signal_data["tickOffset"] = tck_offset
            msg["DATA"].append(signal_data)
        
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_setSignalPhasePlanTicks", res["TYPE"]
        return res


    # Dynamically add one or more zones at given coordinates.
    # x, y: map coordinates; z: elevation (default 0.0);
    # capacity: zone capacity; zone_type: zone type int;
    # transform_coord: set True if coords need network CRS transform.
    # Returns assigned zone IDs.
    def add_zone(self, x, y, capacity, zone_type, z=0.0, transform_coord=False):
        msg = {"TYPE": "CTRL_addZone", "DATA": []}
        if not isinstance(x, list):
            x = [x]
            y = [y]
            z = [z]
            capacity = [capacity]
            zone_type = [zone_type]
        if not isinstance(z, list):
            z = [z] * len(x)
        if not isinstance(transform_coord, list):
            transform_coord = [transform_coord] * len(x)
        assert len(x) == len(y) == len(z) == len(capacity) == len(zone_type), \
            "x, y, z, capacity, and zone_type must have the same length"
        for xi, yi, zi, cap, ztype, tc in zip(x, y, z, capacity, zone_type, transform_coord):
            msg["DATA"].append({"x": xi, "y": yi, "z": zi, "transformCoord": tc, "capacity": cap, "type": ztype})
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_addZone", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res

    # Dynamically add one or more roads and generated lanes.
    # centerline: [[x, y], ...] or [[x, y, z], ...] per road.
    # upstream_road/downstream_road: road orig_id string or list of orig_id strings.
    # parking_capacity: optional road-level parking slots.
    # Pass roads=[{...}] to send fully formed simulator records directly.
    def add_roads(self, centerline=None, upstream_road=None, downstream_road=None,
                  orig_id=None, road_type=None, control_type=None,
                  upstream_control_type=None, downstream_control_type=None,
                  num_lanes=1, lane_width=None, transform_coord=False, roads=None,
                  parking_capacity=None):
        msg = {"TYPE": "CTRL_addRoads", "DATA": []}

        if roads is not None:
            msg["DATA"] = _as_list(roads)
        else:
            if centerline is None:
                raise ValueError("centerline is required when roads is not provided")
            if upstream_road is None or downstream_road is None:
                raise ValueError("upstream_road and downstream_road are required")

            centerlines = [centerline] if _looks_like_centerline(centerline) else list(centerline)
            if len(centerlines) == 0:
                raise ValueError("At least one road centerline is required")

            count = len(centerlines)
            orig_ids = _broadcast(orig_id, count)
            upstream_roads = _broadcast(upstream_road, count)
            downstream_roads = _broadcast(downstream_road, count)
            road_types = _broadcast(road_type, count)
            control_types = _broadcast(control_type, count)
            upstream_control_types = _broadcast(upstream_control_type, count)
            downstream_control_types = _broadcast(downstream_control_type, count)
            lane_counts = _broadcast(num_lanes, count)
            lane_widths = _broadcast(lane_width, count)
            transform_coords = _broadcast(transform_coord, count)
            parking_capacities = _broadcast(parking_capacity, count)

            for cl, oid, up, down, r_type, c_type, up_control, down_control, lane_count, width, tc, pcap in zip(
                    centerlines, orig_ids, upstream_roads, downstream_roads, road_types,
                    control_types, upstream_control_types, downstream_control_types,
                    lane_counts, lane_widths, transform_coords, parking_capacities):
                record = {
                    "centerline": cl,
                    "numLanes": lane_count,
                    "transformCoord": tc,
                }
                if oid is not None:
                    record["origID"] = oid
                _set_road_reference(record, "upStream", up)
                _set_road_reference(record, "downStream", down)
                if r_type is not None:
                    record["roadType"] = r_type
                if c_type is not None:
                    record["controlType"] = c_type
                if up_control is not None:
                    record["upStreamControlType"] = up_control
                if down_control is not None:
                    record["downStreamControlType"] = down_control
                if width is not None:
                    record["laneWidth"] = width
                if pcap is not None:
                    record["parkingCapacity"] = pcap
                msg["DATA"].append(record)

        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_addRoads", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res

    def remove_zone(self, zoneID):
        """Dynamically remove one or more zones by internal zone ID."""
        msg = {"TYPE": "CTRL_removeZone", "DATA": _as_list(zoneID)}
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_removeZone", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res

    def remove_road(self, roadID):
        """Dynamically remove one or more roads by SUMO/original road ID."""
        msg = {"TYPE": "CTRL_removeRoad", "DATA": _as_list(roadID)}
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_removeRoad", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res

    # Dynamically add one or more charging stations at given coordinates.
    # num_l2/l3/bus: charger counts; price_l2/l3: per-unit prices;
    # z: elevation (default 0.0);
    # transform_coord: set True if coords need network CRS transform.
    # Returns assigned (negative) station IDs.
    def add_charging_station(self, x, y, num_l2, num_l3, num_bus, price_l2, price_l3, z=0.0, transform_coord=False):
        msg = {"TYPE": "CTRL_addChargingStation", "DATA": []}
        if not isinstance(x, list):
            x = [x]
            y = [y]
            z = [z]
            num_l2 = [num_l2]
            num_l3 = [num_l3]
            num_bus = [num_bus]
            price_l2 = [price_l2]
            price_l3 = [price_l3]
        if not isinstance(z, list):
            z = [z] * len(x)
        if not isinstance(transform_coord, list):
            transform_coord = [transform_coord] * len(x)
        assert len(x) == len(y) == len(z) == len(num_l2) == len(num_l3) == len(num_bus) == len(price_l2) == len(price_l3), \
            "All positional arguments must have the same length"
        for xi, yi, zi, nl2, nl3, nbus, pl2, pl3, tc in zip(x, y, z, num_l2, num_l3, num_bus, price_l2, price_l3, transform_coord):
            msg["DATA"].append({"x": xi, "y": yi, "z": zi, "transformCoord": tc,
                                "numL2": nl2, "numL3": nl3, "numBus": nbus,
                                "priceL2": pl2, "priceL3": pl3})
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_addChargingStation", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res

    def remove_charging_station(self, stationID):
        """Dynamically remove one or more charging stations by station ID."""
        msg = {"TYPE": "CTRL_removeChargingStation", "DATA": _as_list(stationID)}
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_removeChargingStation", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res

    remove_chargingStation = remove_charging_station

    # Spawn e-taxis parked at given zone(s).
    # zoneID: zone ID or list of zone IDs; num: number of taxis to spawn per zone.
    # Returns spawned vehicle IDs grouped by zone.
    def add_taxi(self, zoneID, num):
        msg = {"TYPE": "CTRL_addTaxi", "DATA": []}
        if not isinstance(zoneID, list):
            zoneID = [zoneID]
        if not isinstance(num, list):
            num = [num] * len(zoneID)
        assert len(zoneID) == len(num), "zoneID and num must have the same length"
        for zid, n in zip(zoneID, num):
            msg["DATA"].append({"zoneID": zid, "num": n})
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_addTaxi", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res

    # Spawn e-buses on existing named route(s).
    # routeName: route name or list of route names; num: buses to spawn per route.
    # Returns spawned vehicle IDs grouped by route.
    def add_bus(self, routeName, num):
        msg = {"TYPE": "CTRL_addBus", "DATA": []}
        if not isinstance(routeName, list):
            routeName = [routeName]
        if not isinstance(num, list):
            num = [num] * len(routeName)
        assert len(routeName) == len(num), "routeName and num must have the same length"
        for rname, n in zip(routeName, num):
            msg["DATA"].append({"routeName": rname, "num": n})
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_addBus", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res

    # Command vehicle(s) to interrupt current activity and go charge.
    # veh_type: True = private EV, False = public taxi.
    # charger_type: ChargingStation.L2 / L3 / BUS.
    # cs_id: 0 = auto-select (nearest for public taxis/buses, cheapest usable
    # station for private EVs); nonzero int = specific station ID.
    # After charging the vehicle returns to its pre-charging destination.
    def go_charging(self, vehID, veh_type, charger_type, cs_id=0):
        msg = {"TYPE": "CTRL_goCharging", "DATA": []}
        if not isinstance(vehID, list):
            vehID = [vehID]
        if not isinstance(veh_type, list):
            veh_type = [veh_type] * len(vehID)
        if not isinstance(charger_type, list):
            charger_type = [charger_type] * len(vehID)
        if not isinstance(cs_id, list):
            cs_id = [cs_id] * len(vehID)
        assert len(vehID) == len(veh_type) == len(charger_type) == len(cs_id), \
            "vehID, veh_type, charger_type, and cs_id must have the same length"
        for vid, vtype, ctype, csid in zip(vehID, veh_type, charger_type, cs_id):
            msg["DATA"].append({"vehID": vid, "vehType": vtype, "chargerType": ctype, "csID": csid})
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_goCharging", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res
     
    
    # reset the simulation with a property file
    def reset(self):
        msg = {"TYPE": "CTRL_reset"}
        res = self.send_receive_msg(msg, ignore_heartbeats=True, max_attempts=-1)

        assert res["TYPE"] == "CTRL_reset", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]

        if "TICK" in res or "tick" in res:
            self.current_tick = int(res.get("TICK", res.get("tick")))
        else:
            self.current_tick = -1
            self.tick()
            assert self.current_tick == 0

        # if viz is running, stop and restart it
        if self.viz_server is not None:
            self.stop_viz()

            time.sleep(1) # wait for five secs if start viz

            self.start_viz()

    # save the simulation instance to zip
    def save(self, filename):
        msg = {"TYPE": "CTRL_save", "DATA": {"path": filename}}
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        
        assert res["TYPE"] == "CTRL_save", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res

    def load(self, filename, reload_network=True):
        """Restore the simulator from a saved snapshot.

        Parameters
        ----------
        filename : str
            Path to the ``.zip`` snapshot previously written by :meth:`save`.
        reload_network : bool
            When ``True`` the simulator fully rebuilds the road
            network and all facilities from the archive.  When ``False`` the
            simulator skips network and facility reconstruction and only
            restores agent state, which is substantially faster when the
            snapshot was taken from the same running instance (e.g. to replay
            an experiment from a preheated checkpoint).  The response includes
            a ``fastLoad`` field confirming whether fast restoration was used.
        """
        msg = {"TYPE": "CTRL_load", "DATA": {"path": filename, "reloadNetwork": bool(reload_network)}}
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_load", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        if "TICK" in res or "tick" in res:
            self.current_tick = int(res.get("TICK", res.get("tick")))
        else:
            synced_tick = self.query_tick()
            self.current_tick = int(synced_tick)
        return res

    # terminate the simulation
    def terminate(self):
        msg = {"TYPE": "CTRL_end"}
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_end", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        self.close()
    
    # close the client but keep the simulator running
    def close(self):
        if self.ws is not None:
            self.ws.close()
            self.ws = None
            self.state = "closed"

        if self.viz_server is not None:
            self.stop_viz()


    def latest_trajectory_output_dir(self, trajectory_output_dir=None, prefer_binary=True, wait_seconds=0):
        """Return the newest trajectory output directory for visualization.

        The latest METS-R SIM writes binary trajectory chunks with a
        ``manifest.json`` file by default. Older runs may still write JSON
        chunks, so this method accepts both formats and prefers binary output
        when available.
        """
        if not self.sim_folder:
            raise ValueError("sim_folder is required to locate trajectory output")

        if trajectory_output_dir is not None:
            roots = [_resolve_trajectory_root(self.sim_folder, trajectory_output_dir)]
        else:
            roots = _configured_trajectory_roots(self.sim_folder)

        deadline = time.time() + wait_seconds
        while True:
            candidates = []
            for root in roots:
                latest = _latest_trajectory_directory(root, prefer_binary=prefer_binary)
                if latest is not None:
                    candidates.append(latest)

            if candidates:
                if prefer_binary:
                    binary_candidates = [
                        directory for directory in candidates
                        if _trajectory_format_score(directory) >= 2
                    ]
                    if binary_candidates:
                        return max(binary_candidates, key=os.path.getmtime)
                return max(candidates, key=os.path.getmtime)

            if wait_seconds <= 0 or time.time() >= deadline:
                roots_text = ", ".join(root for root in roots if root)
                raise FileNotFoundError(
                    "No trajectory output directory found under " + roots_text
            )
            time.sleep(0.5)

    def get_trajectory_manifest(self, trajectory_output_dir=None, prefer_binary=True, wait_seconds=0):
        """Return the manifest for the latest binary trajectory output.

        METS-R SIM binary trajectory format v6 writes zone and charging-station
        frames as sparse deltas. This helper exposes the manifest, including
        ``sparseFrameGroups`` and ``sparseFrameGroupMode``, without making
        callers locate or parse ``manifest.json`` manually.
        """
        latest_directory = self.latest_trajectory_output_dir(
            trajectory_output_dir=trajectory_output_dir,
            prefer_binary=prefer_binary,
            wait_seconds=wait_seconds,
        )
        manifest = _read_trajectory_manifest(latest_directory)
        if manifest is None:
            raise FileNotFoundError(
                "No manifest.json found in trajectory output directory: "
                + latest_directory
            )

        manifest = dict(manifest)
        manifest["_directory"] = latest_directory
        manifest["_manifest_path"] = os.path.join(latest_directory, "manifest.json")
        return manifest

    def get_trajectory_summary(self, trajectory_output_dir=None, prefer_binary=True, wait_seconds=0):
        """Return high-level metadata for the latest trajectory output."""
        latest_directory = self.latest_trajectory_output_dir(
            trajectory_output_dir=trajectory_output_dir,
            prefer_binary=prefer_binary,
            wait_seconds=wait_seconds,
        )
        manifest = _read_trajectory_manifest(latest_directory)
        if manifest is not None:
            return _trajectory_manifest_summary(latest_directory, manifest)

        return {
            "directory": latest_directory,
            "format": _trajectory_format_name(latest_directory),
            "version": None,
            "sparse_frame_groups": [],
            "sparse_frame_group_mode": None,
            "has_sparse_frame_groups": False,
            "has_sparse_zone_frames": False,
            "has_sparse_charging_station_frames": False,
            "has_zone_attributes": False,
            "has_charging_station_attributes": False,
            "has_split_energy_fields": False,
        }

    # open visualization server
    def start_viz(self, trajectory_output_dir=None, server_port=8000, prefer_binary=True, wait_seconds=0):
        latest_directory = self.latest_trajectory_output_dir(
            trajectory_output_dir=trajectory_output_dir,
            prefer_binary=prefer_binary,
            wait_seconds=wait_seconds,
        )

        if self.viz_server is not None:
            self.stop_viz()

        print(
            f"Starting visualization server for {_trajectory_format_name(latest_directory)} "
            f"trajectory output: {latest_directory}"
        )
        self.viz_event, self.viz_server = run_visualization_server(latest_directory, server_port)
        self.viz_port = server_port

    def stop_viz(self):
        if self.viz_server is not None:
            stop_visualization_server(self.viz_event, self.viz_server, self.viz_port or 8000)
        self.viz_event = None
        self.viz_server = None
        self.viz_port = None
    
    def _logMessage(self, direction, msg):
        self._messagesLog.append(
            (datetime.now().strftime("%Y-%m-%d %H:%M:%S"), direction, tuple(msg.items()))
        )
        print(self._messagesLog[-1])
        
    # override __str__ for logging 
    def __str__(self):
        s = f"-----------\n" \
            f"Client INFO\n" \
            f"-----------\n" \
            f"output folder :\t {self.sim_folder}\n" \
            f"address :\t {self.uri}\n" \
            f"state :\t {self.state}\n" 
        return s
