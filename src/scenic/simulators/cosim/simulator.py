from scenic.core.simulators import Simulation, Simulator
from scenic.core.vectors import Orientation, Vector
from scenic.syntax.veneer import verbosePrint
from scenic.simulators.metsr.client import METSRClient
import scenic.simulators.carla.utils.utils as utils
from scenic.simulators.cosim.utils.utils import *
from scenic.core.regions import CircularRegion

from scenic.domains.driving.roads import Lane, Intersection


import pygame
import warnings
import os
import math
import numpy as np
from scenic.core.simulators import SimulationCreationError
import scenic.simulators.cosim.utils.utils as _utils

from scenic.domains.driving.simulators import DrivingSimulation, DrivingSimulator

from scenic.core.distributions import Uniform 

import scenic.simulators.carla.utils.visuals as visuals
from scenic.simulators.carla.blueprints import oldBlueprintNames
from shapely.geometry import Point
import re


try:
    import carla
except ImportError as e:
    raise ModuleNotFoundError('CARLA scenarios require the "carla" Python package') from e

# def initialize_Carla(map_name=None, map_path=None, address="127.0.0.1",port=2000,timeout=10):
#     verbosePrint(f"Connecting to CARLA on port {port}")
#     client = carla.Client(address,port)


class CosimSimulator(DrivingSimulator):
    def __init__(self, 
        metsr_map, 
        carla_map,
        map_path,
        xml_map,
        bubble_size = 50, # Might be good to add some logic for what a minimal bubble size is so users cannot make it too small
        address="127.0.0.1", 
        carla_port=2000, 
        metsr_host="localhost", # Not sure what this actually means here
        metsr_port=4000, 
    #  timestep=1,# Not entirely sure what the distinction between timestep and sim_timestep is in metsr
        timestep=0.1,
        traffic_manager_port=None,
        timeout=10,
        verbose=False,
        render=True,
        record=""
    ):
        super().__init__()


        self.metsr_map_name = metsr_map
        self.timestep = timestep
        self.sim_timestep = timestep
        self.map_path = map_path
        self.bubble_size = bubble_size
        self.render= render
        self.record = record

        # Setting up the Carla Simulator
        verbosePrint(f"Connection to CARLA on port {carla_port}")
        self.carla_client = carla.Client(address,carla_port)
        self.carla_client.set_timeout(timeout)
        """
        Need to figure out how to handle the map paths for this
        """
        if carla_map is not None:
            try:
                self.world = self.carla_client.load_world(carla_map)
                self.xml_to_xodr_map = _utils.generate_map(str(xml_map)) #covert pathlib obj to str for XML tree TODO what is best practice? 
            except Exception as e:
                raise RuntimeError(f"CARLA could not load world '{carla_map}'") from e
        else:
            #TODO figure out how to properly do the map handling here 
            if str(map_path).endswith(".xodr"):
                with open(map_path) as odr_file:
                    self.world = self.carla_client.generate_opendrive_world(odr_file.read())
            else:
                raise RuntimeError("CARLA only supports OpenDrive maps")
        self.timestep = timestep
        if traffic_manager_port is None:
            traffic_manager_port = carla_port + 6000
            assert traffic_manager_port != metsr_port, f"Specified Traffic manager port {traffic_manager_port} is not available"
        self.tm = self.carla_client.get_trafficmanager(traffic_manager_port)
        self.tm.set_synchronous_mode(True)

        settings = self.world.get_settings()
        settings.synchronous_mode = True
        assert timestep <= .1 , f"timestep must be less that 0.1"
        settings.fixed_delta_seconds = timestep
        self.world.apply_settings(settings)
        verbosePrint("Map loaded in simulator.")

        # self.render = render
        # self.record = record 
        # self.scenario_numver = 0
        verbosePrint("Carla was initialized correctly proceeding to Metsr")

        # Setting up Metsr simulator 

        self.metsr_client = METSRClient(host=metsr_host, port=metsr_port, verbose=verbose)

        verbosePrint("Clients have successfully been initialized")

    def createSimulation(self,scene,*, timestep, **kwargs): #TODO: fix timestep
        if timestep is not None and timestep != self.timestep:
            raise RuntimeError(
                "cannot customize timestep for individual CARLA simulations; "
                "set timestep when creating the CarlaSimulator instead"
            )
        return CosimSimulation(
            scene=scene,
            carla_client=self.carla_client,
            metsr_client=self.metsr_client,
            sim_timestep=self.sim_timestep,
            tm=self.tm,
            bubble_size=self.bubble_size,
            render=self.render,
            record=self.record,
            mappings=self.xml_to_xodr_map,
            **kwargs,
        )
    def destroy(self):
        self.metsr_client.close()
        super().destroy()
        settings = self.world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        self.world.apply_settings(settings)
        self.tm.set_synchronous_mode(False)

class CosimSimulation(DrivingSimulation):
    def __init__(self, scene, carla_client, metsr_client, sim_timestep, tm,render,record,mappings, bubble_size=100, **kwargs ):
    
        # Carla and metrs simulators
        self.carla_client = carla_client
        self.metsr_client = metsr_client
        self.sim_timestep = sim_timestep
        
        # Initializing CARLA params
        self.tm = tm # Carla Traffic manager
        self.carla_world = self.carla_client.get_world()
        self.map = self.carla_world.get_map()
        self.blueprintLib = self.carla_world.get_blueprint_library()
        self.carla_cameraManager = None
        self.render = render
        self.record = record

        # Initializing METSR params
        self.next_pv_id = 0
        self.pv_id_map = {}
        self.frozen_vehicles = set()
        self.xodr_to_xml_map = mappings

        self._client_calls = []
        self.count = 0

        # CoSim related params
        self.bubble_size = bubble_size
        self.workspace = scene.workspace
        self.carla_control_lanes = {}
        self.queued_vehicles = {}

        super().__init__(scene, timestep=sim_timestep, **kwargs)



    def setup(self) -> None:
        """
        Setup the simulation instance
            Set initial simulator instance for each object
        """
        self.metsr_client.reset("Data.properties.CARLA") # Not entirley sure what this is for

        weather = self.scene.params.get("weather")
        if weather is not None:
            if isinstance(weather, str):
                self.carla_world.set_weather(getattr(carla.WeatherParameters, weather))
            elif isinstance(weather, dict):
                self.carla_world.set_weather(carla.WeatherParameters(**weather))

        # Setup HUD
        if self.render:
            self.displayDim = (1280, 720)
            self.displayClock = pygame.time.Clock()
            self.camTransform = 0
            pygame.init()
            pygame.font.init()
            self.hud = visuals.HUD(*self.displayDim)
            self.display = pygame.display.set_mode(
                self.displayDim, pygame.HWSURFACE | pygame.DOUBLEBUF
            )
            self.cameraManager = None

        if self.record:
            if not os.path.exists(self.record):
                os.mkdir(self.record)
            name = "{}/scenario{}.log".format(self.record, self.scenario_number)
            # Carla is looking for an absolute path, so convert it if necessary.
            name = os.path.abspath(name)
            self.carla_client.start_recorder(name)

        # Create objects.
        super().setup()

        #TEMP FIX? IDK...
        for object in self.objects[1:]:
            object.carla_actor_flag = False
            object.spawn_guard = 0
        self.objects[0].carla_actor_flag = True
        self.objects[0].spawn_guard = 0

        # print(f"bubble_size {self.bubble_size}")
        self.objects[0].bubble = CircularRegion(center=[self.objects[0].x,
                                                        self.objects[0].y],
                                                        radius=self.bubble_size)
        
        for obj in self.objects:
            if isinstance(obj.carlaActor, carla.Vehicle):
                obj.carlaActor.apply_control(
                    carla.VehicleControl(manual_gear_shift=False)
                )
        self.carla_world.tick()

        # Set up camera manager and collision sensor for ego
        if self.render:
            camIndex = 0
            camPosIndex = 0
            egoActor = self.objects[0].carlaActor
            self.cameraManager = visuals.CameraManager(self.carla_world, egoActor, self.hud)
            self.cameraManager._transform_index = camPosIndex
            self.cameraManager.set_sensor(camIndex)
            self.cameraManager.set_transform(self.camTransform)

        self.carla_world.tick()  ## allowing manualgearshift to take effect    # TODO still need this?

        for obj in self.scene.objects:
            if obj.carla_actor_flag:
                if obj.speed is not None and obj.speed != 0:
                    raise RuntimeError(
                        f"object {obj} cannot have a nonzero initial speed "
                        "(this is not yet possible in CARLA)"
                    )
                
        self.synchronize_clients()


    def createObjectInMetsr(self, obj) -> None:
        """
        Create object in Metsr
        """
        assert obj.origin, "Metsr objects must have a defined origin"
        assert obj.destination, "Metsr objects must have a defined destination"

        print(f"Creating obj: {obj} with position: {obj.position} in METSR")

        call_kwargs = {
            "vehID": self.getMetsrPrivateVehId(obj),
            "origin": obj.origin,
            "destination": obj.destination,
        }

        self.metsr_client.generate_trip(**call_kwargs)


    def createObjectInCarla(self,obj) -> None:
        # Extract blueprint
        print(f"Creating obj: {obj} with position: {obj.position} in CARLA")
        try:
            blueprint = self.blueprintLib.find(obj.blueprint)
        except IndexError as e:
            found = False
            if obj.blueprint in oldBlueprintNames:
                for oldName in oldBlueprintNames[obj.blueprint]:
                    try:
                        blueprint = self.blueprintLib.find(oldName)
                        found = True
                        warnings.warn(
                            f"CARLA blueprint {obj.blueprint} not found; "
                            f"using older version {oldName}"
                        )
                        obj.blueprint = oldName
                        break
                    except IndexError:
                        continue
            if not found:
                raise SimulationCreationError(
                    f"Unable to find blueprint {obj.blueprint}" f" for object {obj}"
                ) from e
        if obj.rolename is not None:
            blueprint.set_attribute("role_name", obj.rolename)

        # set walker as not invincible
        if blueprint.has_attribute("is_invincible"):
            blueprint.set_attribute("is_invincible", "False")

        # Set up transform
        loc = utils.scenicToCarlaLocation(
            obj.position,
            world=self.carla_world,
            blueprint=obj.blueprint,
            snapToGround=obj.snapToGround,
        )
        rot = utils.scenicToCarlaRotation(obj.orientation)
        transform = carla.Transform(loc, rot)

        # Color, cannot be set for Pedestrians
        if blueprint.has_attribute("color") and obj.color is not None:
            c = obj.color
            c_str = f"{int(c.r*255)},{int(c.g*255)},{int(c.b*255)}"
            blueprint.set_attribute("color", c_str)

        # Create Carla actor
        # print(f"Spawning actor {obj} in location {loc} with original pos: {obj.position}")
        try:
            carlaActor = self.carla_world.spawn_actor(blueprint, transform)
        except Exception as e:
            print(f"Error: {e} occured \n displaying object positions")
            for obj in self.objects:
                car_data = self.metsr_client.query_vehicle(self.getMetsrPrivateVehId(obj), True, True)
                simulator = "carla" if obj.carla_actor_flag else "metsr"
                print(f"{obj} in {simulator}: [Metsr][Scenic] coords X: {car_data['DATA'][0]['x'], obj.position.x}, y:{car_data['DATA'][0]['y'], obj.position.y}")
                print(f"Query Vehicle resuts for obj: :{obj}, is: {car_data['DATA']}")
            # raise e(f"Error : {e} occured")
        # if carlaActor is None:
        #     raise SimulationCreationError(f"Unable to spawn object {obj}")
        obj.carlaActor = carlaActor

        carlaActor.set_simulate_physics(obj.physics)

        if isinstance(carlaActor, carla.Vehicle):
            # TODO should get dimensions at compile time, not simulation time
            extent = carlaActor.bounding_box.extent
            ex, ey, ez = extent.x, extent.y, extent.z
            # Ensure each extent is positive to work around CARLA issue #5841
            obj.width = ey * 2 if ey > 0 else obj.width
            obj.length = ex * 2 if ex > 0 else obj.length
            obj.height = ez * 2 if ez > 0 else obj.height
            carlaActor.apply_control(carla.VehicleControl(manual_gear_shift=True, gear=1))
        elif isinstance(carlaActor, carla.Walker):
            carlaActor.apply_control(carla.WalkerControl())
            # spawn walker controller
            controller_bp = self.blueprintLib.find("controller.ai.walker")
            controller = self.carla_world.try_spawn_actor(
                controller_bp, carla.Transform(), carlaActor
            )
            if controller is None:
                raise SimulationCreationError(
                    f"Unable to spawn carla controller for object {obj}"
                )
            obj.carlaController = controller
   
    

    def createObjectInSimulator(self, obj) -> None:
        """
        Create object in the corresponding simulator according to its flag
        """
        assert obj.origin,      "All objects must have an origin"
        assert obj.destination, "All objects must have an destination"
        
        assert hasattr(obj, "carla_actor_flag"), "All objects must have attribute: carla_actor_flag"

        # try:
        #     vehID = self.getMetsrPrivateVehId(obj)
        #     veh_data = self.metsr_client.query_vehicle(vehID, True, True)
        #     if 'x' in veh_data[0] and 'y' in veh_data[0]:

        
        if obj == self.objects[0]:
            # Special handling creating EGO in both simulators
            # ensures consistency for obje queue spawning in METSR
            self.createObjectInCarla(obj)
            self.createObjectInMetsr(obj)
        elif obj.carla_actor_flag:
            self.createObjectInCarla(obj)
        else:
            self.createObjectInMetsr(obj)


    def getCarlaProperties(self, obj, properties) -> dict[str, float | Vector | int]:
        """
        Return simulator specific properties for object in CARLA simulator
        """
        # Extract Carla properties
        carlaActor = obj.carlaActor
        currTransform = carlaActor.get_transform()
        currLoc = currTransform.location
        currRot = currTransform.rotation
        currVel = carlaActor.get_velocity()
        currAngVel = carlaActor.get_angular_velocity()

        # Prepare Scenic object properties
        position = utils.carlaToScenicPosition(currLoc)
        velocity = utils.carlaToScenicPosition(currVel)
        speed = math.hypot(*velocity)
        angularSpeed = utils.carlaToScenicAngularSpeed(currAngVel)
        angularVelocity = utils.carlaToScenicAngularVel(currAngVel)
        globalOrientation = utils.carlaToScenicOrientation(currRot)
        yaw, pitch, roll = obj.parentOrientation.localAnglesFor(globalOrientation)
        elevation = utils.carlaToScenicElevation(currLoc)

        values = dict(
            position=position,
            velocity=velocity,
            speed=speed,
            angularSpeed=angularSpeed,
            angularVelocity=angularVelocity,
            yaw=yaw,
            pitch=pitch,
            roll=roll,
            elevation=elevation,
        )
        # print(f"{obj} properties: {values}")
        return values


    def getMetsrProperties(self, obj, properties) -> dict[str, float | Vector | int]:
        """
        Return simulator specific properties for object in Metsr
        """
        if obj in self.frozen_vehicles:
            return None
        
        raw_data = self.obj_data_cache[obj]

        if "road" not in raw_data and raw_data["state"] <=0:
            self.frozen_vehicles.add(obj)

        position = Vector(raw_data["x"], raw_data["y"], 0)
        speed = raw_data["speed"]
        bearing = math.radians(raw_data["bearing"])
        globalOrientation = Orientation.fromEuler(bearing,0,0)
        yaw, pitch, roll = obj.parentOrientation.localAnglesFor(globalOrientation)
        velocity = Vector(0, speed, 0).rotatedBy(yaw)
        angularSpeed = 0
        angularVelocity = Vector(0,0,0)

        values = dict(
            position=position,
            velocity=velocity,
            speed=speed,
            angularSpeed=angularSpeed,
            angularVelocity=angularVelocity,
            yaw=yaw,
            pitch=pitch,
            roll=roll,
            elevation=float(0)
        )
        return values

    def getProperties(self, obj, properties)-> dict[str, float | Vector | int]:
        """
        Return properties for any simulator object
        """
        assert hasattr(obj, "carla_actor_flag"), f"Object is not assigned properly to a simulator instance"
        """ TODO: Sometimes update objects fails on the first step -- Not clear why this occurs -- adding a check before accessing obj.carlaActor """
        # if obj.carla_actor_flag: 
        #     if obj.carlaActor == None: # TODO added check here to ensure actor is created before polling simulator
        #         print(f"Something strange has occured for obj {obj} at step: {self.count}")
        #     properties = self.getCarlaProperties(obj,properties)
        # else:
        properties =  self.getMetsrProperties(obj,properties)
        return properties

    def getMetsrPrivateVehId(self,obj) -> int:
        """
        Return unique vehicle idea
            Generates a new ID if none exists for vehicle
        """
        if obj not in self.pv_id_map:
            self.pv_id_map[obj] = self.next_pv_id
            self.next_pv_id += 1
        return self.pv_id_map[obj]


    def step(self) -> None:
        """
        Step both simulators -> Update ego bubble 
            Update actor locations in either bubble

            TODO : Verify fixed! - Seems like carla needs to be stepped after creating a new object
            TODO : Verify fixed! - If a NON-ego car spawns in the bubble (@ step 0) getproperties fails (carlaActor == None) 
            TODO : METSR vrs CARLA deviation METSR queue spawning occupied CARLA position 

        """

        lanes, new_lanes, old_lanes = self.update_carla_lanes()
        self.release_lanes(old_lanes)
        self.freeze_lanes(new_lanes)

        # Update state of simulator with new bubble region
        self.update_bubble_objects(lanes,new_lanes)

        #Carla step
        self.carla_world.tick() 
        self.synchronize_clients()
        # Need to ensure carla is ticked a
        for obj in self.objects: 
            if obj.spawn_guard > 0:
                obj.spawn_guard -= 1

        if self.render:
            self.cameraManager.render(self.display)
            pygame.display.flip()
       
       # Metsr step
        self.count += 1
        if self.count % 100 == 0: # just removing for now
            print(".", end="", flush=True)
        self.metsr_client.tick()

        # Generate bubble region based on ego objects 
        # TODO update logic for multiple high-fidelity zones
        self.objects[0].bubble = CircularRegion(center=[self.objects[0].x,
                                                        self.objects[0].y],
                                                        radius=self.bubble_size)


        self.synchronize_clients()
        # for obj in self.objects:
        #     if obj.carla_actor_flag:
        #         loc = obj.carlaActor.get_location()
        #         car_data = self.metsr_client.query_vehicle(self.getMetsrPrivateVehId(obj), True, True)
        #         if not math.isclose(car_data['DATA'][0]['x'], loc.x) or not math.isclose(car_data['DATA'][0]['y'],-loc.y):
        #             print(f"{car_data['DATA'][0]['x'], loc.x} -- {car_data['DATA'][0]['y'], -loc.y}")


        # Set intersections
        intersections = self.get_carla_intersections()
        if intersections != []:
            self.synchronize_signals(intersections)
        

    def get_carla_lanes(self) -> list[Lane]:
        """
        Collect the current set of lanes intersecting the CoSimulation bubble
        """
        # Set ego, lane, bubble
        ego = self.objects[0]
        ego_lane = ego._lane
        bubble_region = ego.bubble 

        # TODO what to do if the actor is not currently in a lane
        if ego_lane is None:
            lanes = [*self.workspace.network.lanes]            
            distances = [(lane.distanceTo(ego.position),lane) for lane in lanes]
            ego_lane = min(distances, key=lambda t: t[0])[1] # min distance over all lanes
        
        # Collect lanes which intersect bubble
        carla_lanes = []
        for lane in self.workspace.network.lanes:
            if lane.intersects(bubble_region):
                carla_lanes.append(lane) 

        return carla_lanes
    
    def get_carla_intersections(self) -> list[Intersection] | None:
        """
        TODO: will need update for multiple high-fidelity zones
        Collect any intersections currently within the bubble
            (1) TODO consider when an intersection should be included 
        """
        carla_intersections = []
        ego = self.objects[0]
        
        # check if ego in intersection
        if ego._intersection:
            carla_intersections.append(ego._intersection)

        # Bubble region
        bubble_region = ego.bubble 

        # Collect intersections which intersect bubble
        for intersection in self.workspace.network.intersections:
            if intersection.intersects(bubble_region):
                carla_intersections.append(intersection)
            
        return carla_intersections
    
    def _set_intersections(self) -> None:
        """
        Checks if an intersection is fully inclosed in bubble
            (1) If all connecting roads for an intersection are in bubble so is intersection
        
        """
        pass

    def synchronize_signals(self, intersections: list[Intersection]) -> None:
        """
        Ensures consistency for light signals for all intersections contained in the bubble region
        """
        pass

    def synchronize_clients(self):
        """
        Update Metsr client so object positions are synchronized with the behaviors in Scenic
        """
        carla_actors = [obj for obj in self.objects if obj.carla_actor_flag]
        for obj in carla_actors:
            loc = obj.carlaActor.get_location()
            vehID = self.getMetsrPrivateVehId(obj)
            assert obj._lane, f"Object {obj} is not on a lane"
            roadID = self.get_mapping(obj._lane)
            # print(f"Teleporting Vehicle to location {loc.x, -loc.y}")
            self.metsr_client.teleport_cosim_vehicle(vehID, roadID, loc.x, -loc.y, private_veh = True, transform_coords = True )

        
    


    def update_carla_lanes(self) -> None: #TODO break this out into multiple functions/helpers
        """
            TODO : Avoid nested call structure implement both in step
            Update the state of lanes
                (1): Freeze new lanes inside CoSimulator bubble
                (2): Release lanes outside the CoSimulator bubble
                (3): Update objects inside Carla with update lanes
        """
        # Collect lanes intersecting the bubble
        lanes = self.get_carla_lanes() 
        # Find the corresponding METSR keys
        carla_lane_ids = [self.get_mapping(lane) for lane in lanes] 

        # Filter for uniquenes
        carla_lane_ids = set(carla_lane_ids) 
        # Lanes which are already set
        curr_frozen_ids = list(self.carla_control_lanes.keys())
    
        # Collect new and old lanes 
        new_lanes = [id for id in carla_lane_ids if id not in curr_frozen_ids]
        old_lanes = [id for id in list(self.carla_control_lanes.keys()) if id not in carla_lane_ids]
        
        # Update object existance based on bubble changes
        return lanes, new_lanes,old_lanes

   
   
    def update_bubble_objects(self, carla_lanes: list[str], new_lanes: list[str]) -> None: 
        """
        Check each objects current lane
            (1) If object is in a Carla lane and not created yet
                then create object in Carla
            (2) Object is NOT in a Carla lane but still in Carla
                then delete object in Carla=
        """
        carla_actors = [obj for obj in self.objects if obj.carla_actor_flag] # TODO might need to consider a more efficient data structure here
        # print(f"Updating object for lanes: {carla_lanes}")
        for obj in self.objects[1:]: #TODO Need to get a helper for finding the closest lane when no lane 
            # TODO check for Driving object
            lane = obj._lane
            if lane: 
                # If object is in a lane
                if lane in carla_lanes and not obj.carla_actor_flag: 
                    if lane in new_lanes:
                        print(f"Object in new lane intersecting the bubble")
                    # Object is inside the bubble lanes but not instantiated
                    veh_data = self.metsr_client.query_vehicle(self.getMetsrPrivateVehId(obj), True, True)
                    if 'dist' in veh_data["DATA"][0]:
                        print(f"Creating Object: {obj}, at position: {obj.position} at {self.count}")
                        self.createObjectInCarla(obj)
                        carla_actors.append(obj)
                        obj.carla_actor_flag = True
                        obj.spawn_guard = 2

                elif lane not in carla_lanes and obj.carla_actor_flag:
                    # Object is not in bubble anymore but still in CARLA
                    if obj.spawn_guard == 0:
                        obj.carla_actor_flag = False
                        # print(f"Destroying obj: {obj} at {self.count}")
                        print(f"Destroying Object: {obj}, Lane: {lane.id, lane.uid}, actor_flag {obj.carla_actor_flag}, postion: {obj.position}")
                        self.destroy_carla_obj(obj)
                        carla_actors.remove(obj)

                        
        

        



    def freeze_lanes(self, keys: list[str]) -> None:
        """
            Query carla actors for relevent road segments which metsr will not control
        """
        for key in keys:
            assert key not in self.carla_control_lanes, "Attempted to freeze already frozen lane"
            # Keep track of frozen lanes
            self.carla_control_lanes[key] = True 
            self.metsr_client.set_cosim_road(key)

    def release_lanes(self,keys: list[str]) -> None:
        """
            Release frozen lanes in Carla
        """
        for key in keys:
            assert key in self.carla_control_lanes, "Attempted to release non frozen lane"
            # Remove frozen lane from record
            del self.carla_control_lanes[key] 
            self.metsr_client.release_cosim_road(key)


    def destroy_carla_obj(self,obj) -> None:
        """
        Special destroy method for updating high-fidelity bubble
        """
        if obj.carlaActor is not None:
            if isinstance(obj.carlaActor, carla.Vehicle):
                obj.carlaActor.set_autopilot(False, self.tm.get_port())
            if isinstance(obj.carlaActor, carla.Walker):
                obj.cralaController.stop()
                obj.carlaController.destroy()
            obj.carlaActor.destroy()
            obj.carlaActor = None # Set this to None to prevent reaccess a previously deleted vehicle? 

    
    def get_mapping(self,lane: Lane) -> str:
        """
        TODO: Takes an object and maps the closest lane of the object 
              in xodr to its corresponding xml key
        """
        metsr_key=None
        # Parent road key with associated lane id
        query_key = f'{lane.road.id}_{lane.id}' 
        
        # Check if element is present in map between formats
        if query_key in self.xodr_to_xml_map: 
            metsr_key = self.xodr_to_xml_map[query_key]

        # There must be a valid mapping 
        assert metsr_key is not None, f"Error identifying associated ID for {query_key}" 
        return metsr_key

    
    def metsr_destroy(self) -> None:
        """
        Metsr specific destroy
        """
        if self.metsr_client.verbose:
            print("Client Messages Log:")
            print("[")
            for call in self.client._messagesLog:
                print(f"    {call},")
            print("]")


    def carla_destroy(self) -> None:
        """
        Carla specific destroy
        """
        for obj in self.objects:
            if obj.carlaActor is not None:
                if isinstance(obj.carlaActor, carla.Vehicle):
                    obj.carlaActor.set_autopilot(False, self.tm.get_port())
                if isinstance(obj.carlaActor, carla.Walker):
                    obj.carlaController.stop()
                    obj.carlaController.destroy()
                obj.carlaActor.destroy()
        if self.render and self.cameraManager:
            self.cameraManager.destroy_sensor()

        self.carla_client.stop_recorder()
        # self.carla_world.tick()

    
    def destroy(self) -> None:
        """
        Destroy both simulator instances
        """
        self.metsr_destroy()
        self.carla_destroy()
        super().destroy()


    def _nearest_lane(self,obj) -> Lane | None : # TODO :: Update lane logic to consider intersections
        """
        Docstring for _nearest_lane
        
        Return the nearest lane to the object should ensure all objects are cars? 
        """
        lane = obj._lane
        if lane:
            return lane
        else:
            return None
        
    def executeActions(self, allActions) -> None:
        """
        Apply control updates which were accumulated while executing the actions
        Filters out actions for Carla only objects
        """
        carla_actions = {}
        for obj in self.agents:
            if obj.carla_actor_flag:
                carla_actions[obj] = allActions[obj]
        
        super().executeActions(carla_actions)

        for obj in self.agents:
            if obj.carla_actor_flag:
                ctrl = obj._control
                if ctrl is not None:
                    obj.carlaActor.apply_control(ctrl)
                    obj._control = None
                    

    def updateObjects(self) -> None:
        # metsr_obj = [obj for obj in self.objects if not obj.carla_actor_flag]
        # metsr_obj.append(self.objects[0])
        obj_veh_ids = [self.getMetsrPrivateVehId(obj) for obj in self.objects]
        raw_veh_data = self.metsr_client.query_vehicle(obj_veh_ids, True, True)
        self.obj_data_cache = {obj: raw_veh_data['DATA'][i] for i, obj in enumerate(self.objects)}

        for obj in self.obj_data_cache:
            if 'dist' not in self.obj_data_cache[obj]:
                self.queued_vehicles[obj] = True
            elif 'dist' in self.obj_data_cache[obj] and obj in self.queued_vehicles:
                print(f"obj {obj} leaving the spawn queue")
                del self.queued_vehicles[obj] 

        super().updateObjects()
        self.obj_data_cache = None 