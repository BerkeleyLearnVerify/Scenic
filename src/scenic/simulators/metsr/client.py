import datetime
import json
import time
import threading


from websockets.sync.client import connect


class METSRClient:

    def __init__(self, host, port, sim_folder = None, manager = None,
        max_connection_attempts = 5, timeout = 30, verbose = False):
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
 
        # Track the tick of the corresponding simulator
        self.current_tick = None

        # Establish connection
        failed_attempts = 0
        while True:
            try:
                self.ws = connect(self.uri)
                self.state = "connected"
                if self.verbose:
                    print(f"Connected to {self.uri}")
                break
            except ConnectionRefusedError:
                print(f"Attempt to connect to {self.uri} failed. "
                      f"Waiting for 10 seconds before trying again... "
                      f"({max_connection_attempts - failed_attempts} attempts remaining)")
                failed_attempts += 1
                if failed_attempts >= max_connection_attempts:
                    self.state = "failed"
                    raise RuntimeError("Could not connect to METS-R Sim")
                time.sleep(10)

        # Ensure server is initialized by waiting to receive an initial packet
        # (could be ANS_ready or a heartbeat)
        self.receive_msg(ignore_heartbeats=False)

        self.lock = threading.Lock()

    def send_msg(self, msg):
        if self.verbose:
            self._logMessage("SENT", msg)
        self.ws.send(json.dumps(msg))

    def receive_msg(self, ignore_heartbeats, waiting_forever = True):
        start_time = time.time()
        while True:
            raw_msg = self.ws.recv(timeout = self.timeout)

            # Decode the json string
            msg = json.loads(str(raw_msg))

            if self.verbose:
                self._logMessage("RECEIVED", msg)
            
            # EVERY decoded msg must have a TYPE field
            assert "TYPE" in msg.keys(), "No type field in received message"
            assert msg["TYPE"].split("_")[0] in {"STEP", "ANS", "CTRL", "ATK"}, "Uknown message type: " + str(msg["TYPE"])

            # Allow tick()
            if msg["TYPE"] in {"ANS_ready"}:
                self.current_tick = 0
                continue

            # Return decoded message, if it's not an ignored heartbeat
            if not ignore_heartbeats or msg["TYPE"] != "STEP":
                return msg
            
            if time.time() - start_time > self.timeout and not waiting_forever:
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
                        res = self.receive_msg(ignore_heartbeats=ignore_heartbeats, waiting_forever=False)
                        if num_attempts >= max_attempts:
                            print(f"Failed to receive response after {max_attempts} attempts")
                            break
                    else:
                        res = self.receive_msg(ignore_heartbeats=ignore_heartbeats, waiting_forever=True)
            except KeyboardInterrupt:
                print("\nKeyboardInterrupt detected. Stopping the current operation but keeping the server active.")
                # Reset state or resources if necessary to allow future operations
                return None  # Return None to indicate the operation was interrupted
            except Exception as e:
                print(f"An unexpected error occurred: {e}")
                # Optional: Handle other types of exceptions if needed
            return res

    def tick(self, step_num = 1, wait_forever = False):
        assert self.current_tick is not None, "self.current_tick is None. Reset should be called first"
        msg = {"TYPE": "STEP", "TICK": self.current_tick, "NUM": step_num}
        self.send_msg(msg)

        while True:
            # Move through messages until we get to an up to date heartbeat
            res = self.receive_msg(ignore_heartbeats=False, waiting_forever=wait_forever)

            assert res["TYPE"] == "STEP", res["TYPE"]
            if res["TICK"] == self.current_tick + step_num:
                break

        self.current_tick = res["TICK"]
   
    # QUERY: inspect the state of the simulator
    # By default query public vehicles
    def query_vehicle(self, id = None, private_veh = False, transform_coords = False):
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
 
    # query taxi
    def query_taxi(self, id = None):
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
        
    # query bus
    def query_bus(self, id = None):
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

        
    # query road
    def query_road(self, id = None):
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

    # query zone
    def query_zone(self, id = None):
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

    # query signal
    def query_signal(self, id = None):
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
    
    # query chargingStation
    def query_chargingStation(self, id = None):
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
    
    # query vehicleID within the co-sim road
    def query_coSimVehicle(self):
        my_msg = {"TYPE": "QUERY_coSimVehicle"}
        res = self.send_receive_msg(my_msg, ignore_heartbeats=True)
        assert res["TYPE"] == "ANS_coSimVehicle", res["TYPE"]
        return res
    

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
                "TYPE": "CTRL_releaseCoSimRoad",
                "DATA": [] 
              }
        if not isinstance(roadID, list):
            roadID = [roadID]
        for i in roadID:
            msg['DATA'].append(i)
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_releaseCoSimRoad", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res
        
    # teleport vehicle to a target location specified by road and coordiantes, only work when the road is a cosim road
    def teleport_cosim_vehicle(self, vehID, roadID, x, y, private_veh = False, transform_coords = False):
        msg = {
                "TYPE": "CTRL_teleportCoSimVeh",
                "DATA": []
                }
        if not isinstance(vehID, list):
            vehID = [vehID]
            roadID = [roadID]
            x = [x]
            y = [y]
        if not isinstance(private_veh, list):
            private_veh = [private_veh] * len(vehID)
        if not isinstance(transform_coords, list):
            transform_coords = [transform_coords] * len(vehID)
        for vehID, roadID, x, y, private_veh, transform_coords in zip(vehID, roadID, x, y, private_veh, transform_coords):
            msg["DATA"].append({"vehID": vehID, "roadID": roadID, "x": x, "y": y, "vehType": private_veh, "transformCoord": transform_coords})
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_teleportCoSimVeh", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res
    
    # teleport vehicle to a target location specified by road, lane, and distance to the downstream junction
    def teleport_trace_replay_vehicle(self, vehID, roadID, laneID, dist, private_veh = False):
        msg = {
                "TYPE": "CTRL_teleportTraceReplayVeh",
                "DATA": []
                }
        if not isinstance(vehID, list):
            vehID = [vehID]
            roadID = [roadID]
            laneID = [laneID]
            dist = [dist]
        if not isinstance(private_veh, list):
            private_veh = [private_veh] * len(vehID)
        for vehID, roadID, laneID, dist, private_veh in zip(vehID, roadID, laneID, dist, private_veh):
            msg["DATA"].append({"vehID": vehID, "roadID": roadID, "laneID": laneID, "dist": dist, "vehType": private_veh})
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_teleportTraceReplayVeh", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res
    
    # enter the next road
    def enter_next_road(self, vehID, private_veh = False):
        msg = {
                "TYPE": "CTRL_enterNextRoad",
                "DATA": []
                }
        if not isinstance(vehID, list):
            vehID = [vehID]
        if not isinstance(private_veh, list):
            private_veh = [private_veh] * len(vehID)
        
        for vehID, private_veh in zip(vehID, private_veh):
            msg["DATA"].append({"vehID": vehID, "vehType": private_veh})

        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_enterNextRoad", res["TYPE"]
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
    
    # update the sensor type of specified vehicle
    def update_vehicle_sensor_type(self, vehID, sensorType, private_veh = False):
        msg = {
                "TYPE": "CTRL_updateVehicleSensorType",
                "DATA": []
                }
        if not isinstance(vehID, list):
            vehID = [vehID]
        if not isinstance(private_veh, list):
            private_veh = [private_veh] * len(vehID)
        if not isinstance(sensorType, list):
            sensorType = [sensorType] * len(vehID)
        for vehID, sensorType, private_veh in zip(vehID, sensorType, private_veh):
            msg["DATA"].append({"vehID": vehID, "sensorType": sensorType, "vehType": private_veh})
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_updateVehicleSensorType", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res
    
    # dispatch taxi
    def dispatch_taxi(self, vehID, orig, dest, num):
        msg = {
                "TYPE": "CTRL_dispatchTaxi",
                "DATA": []
                }
        if not isinstance(vehID, list):
            vehID = [vehID]
        if not isinstance(orig, list):
            orig = [orig] * len(vehID)
        if not isinstance(dest, list):
            dest = [dest] * len(vehID)
        if not isinstance(num, list):
            num = [num] * len(vehID)

        for vehID, orig, dest, num in zip(vehID, orig, dest, num):
            msg["DATA"].append({"vehID": vehID, "orig": orig, "dest": dest, "num": num})
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_dispatchTaxi", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res
    
    def dispatch_taxi_between_roads(self, vehID, orig, dest, num):
        msg = {
                "TYPE": "CTRL_dispTaxiBwRoads",
                "DATA": []
                }
        if not isinstance(vehID, list):
            vehID = [vehID]
        if not isinstance(orig, list):
            orig = [orig] * len(vehID)
        if not isinstance(dest, list):
            dest = [dest] * len(vehID)
        if not isinstance(num, list):
            num = [num] * len(vehID)

        for vehID, orig, dest, num in zip(vehID, orig, dest, num):
            msg["DATA"].append({"vehID": vehID, "orig": orig, "dest": dest, "num": num})
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_dispTaxiBwRoads", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res
    
    def add_taxi_requests(self, zoneID, dest, num):
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

        for zoneID, dest, num in zip(zoneID, dest, num):
            msg["DATA"].append({"zoneID": zoneID, "dest": dest, "num": num})
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_addTaxiRequests", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res
    
    def add_taxi_requests_between_roads(self, zoneID, orig, dest, num):
        msg = {
                "TYPE": "CTRL_addTaxiReqBwRoads",
                "DATA": []
                }
        if not isinstance(orig, list):
            orig = [orig]
        if not isinstance(zoneID, list):
            zoneID = [zoneID] * len(orig)
        if not isinstance(dest, list):
            dest = [dest] * len(zoneID)
        if not isinstance(num, list):
            num = [num] * len(zoneID)

        for zoneID, orig, dest, num in zip(zoneID, orig, dest, num):
            msg["DATA"].append({"zoneID": zoneID, "orig": orig, "dest": dest, "num": num})
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_addTaxiReqBwRoads", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res
    

    # assign bus
    def assign_request_to_bus(self, vehID, orig, dest, num):
        msg = {
                "TYPE": "CTRL_assignRequestToBus",
                "DATA": []
                }
        if not isinstance(vehID, list):
            vehID = [vehID]
        if not isinstance(orig, list):
            orig = [orig] * len(vehID)
        if not isinstance(dest, list):
            dest = [dest] * len(vehID)
        if not isinstance(num, list):
            num = [num] * len(vehID)

        for vehID, orig, dest, num in zip(vehID, orig, dest, num):
            msg["DATA"].append({"vehID": vehID, "orig": orig, "dest": dest, "num": num})
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_assignRequestToBus", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res
    
    def add_bus_requests(self, zoneID, dest, num):
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

        for zoneID, dest, num in zip(zoneID, dest, num):
            msg["DATA"].append({"zoneID": zoneID, "dest": dest, "num": num})
        res = self.send_receive_msg(msg, ignore_heartbeats=True)
        assert res["TYPE"] == "CTRL_addBusRequests", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]
        return res
    
    
    # reset the simulation with a property file
    def reset(self, prop_file):
        msg = {"TYPE": "CTRL_reset", "propertyFile": prop_file}
        res = self.send_receive_msg(msg, ignore_heartbeats=True, max_attempts=-1)

        assert res["TYPE"] == "CTRL_reset", res["TYPE"]
        assert res["CODE"] == "OK", res["CODE"]

        self.current_tick = -1
        self.tick()
        assert self.current_tick == 0

        # if viz is running, stop and restart it
        if self.viz_server is not None:
            self.stop_viz()

            time.sleep(1) # wait for five secs if start viz

            self.start_viz()
    
    # reset the simulation with a map name
    def reset_map(self, map_name):
        # find the property file for the map
        if map_name == "CARLA":
            # copy CARLA data in the sim folder
            # source_path = "data/CARLA"
            # specify the property file
            prop_file = "Data.properties.CARLA"
        elif map_name == "NYC":
            # copy NYC data in the sim folder
            # source_path = "data/NYC"
            # specify the property file
            prop_file = "Data.properties.NYC"
        elif map_name == "UA":
            # copy UA data in the sim folder
            # source_path = "data/UA"
            # specify the property file
            prop_file = "Data.properties.UA"

        # docker_cp_command = f"docker cp {source_path} {self.docker_id}:/home/test/data/"
        # subprocess.run(docker_cp_command, shell=True, check=True)
        
        # reset the simulation with the property file
        self.reset(prop_file)

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


    # open visualization server
    def start_viz(self):
        # obtain the latest directory in the sim_folder/trajectory_output
        # get the latest directory
        list_of_files = [os.path.join(self.sim_folder + "/trajectory_output", f) for f in os.listdir(self.sim_folder + "/trajectory_output")]
        # sort the list of files by creation time
        latest_directory = max(list_of_files, key=os.path.getmtime)
        # open the visualization server
        self.viz_event, self.viz_server = run_visualization_server(latest_directory)

    def stop_viz(self):
        if self.viz_server is not None:
            stop_visualization_server(self.viz_event, self.viz_server)
        self.viz_event = None
        self.viz_server = None
    
    def _logMessage(self, direction, msg):
        self._messagesLog.append(
            (datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"), direction, tuple(msg.items()))
        )

    # override __str__ for logging 
    def __str__(self):
        s = f"-----------\n" \
            f"Client INFO\n" \
            f"-----------\n" \
            f"output folder :\t {self.sim_folder}\n" \
            f"address :\t {self.uri}\n" \
            f"state :\t {self.state}\n" 
        return s