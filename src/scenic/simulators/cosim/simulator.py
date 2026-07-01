from scenic.core.simulators import Simulation, Simulator
from scenic.core.vectors import Orientation, Vector
from scenic.syntax.veneer import verbosePrint
from scenic.simulators.metsr.client import METSRClient
from scenic.simulators.cosim.utils.utils import *
from scenic.core.regions import CircularRegion
from scenic.core.object_types import Object
from scenic.core.simulators import SimulationCreationError, ObjectMissingInSimulation
from scenic.domains.driving.roads import Lane, Intersection, Road
from scenic.domains.driving.simulators import DrivingSimulation, DrivingSimulator

import pygame
import warnings
import os
import math
import random
import scenic.simulators.cosim.utils.utils as _utils
import scenic.simulators.carla.utils.utils as utils
import pandas as pd

from .utils.network_helper import network_cache
from .utils.global_route_planner import GlobalRoutePlanner

import scenic.simulators.carla.utils.visuals as visuals
from scenic.simulators.carla.blueprints import oldBlueprintNames

try:
    import carla
except ImportError as e:
    raise ModuleNotFoundError('CARLA scenarios require the "carla" Python package') from e

class CosimSimulator(DrivingSimulator):
    def __init__(self, 
        carla_map,
        map_path,
        xml_map,
        bubble_size = 50, # Might be good to add some logic for what a minimal bubble size is so users cannot make it too small
        address="127.0.0.1", 
        carla_port=2000, 
        metsr_host="localhost", # Not sure what this actually means here
        metsr_port=4000, 
        timestep=0.1, # Not entirely sure what the distinction between timestep and sim_timestep is in metsr
        sim_timestep=0.1,
        traffic_manager_port=None,
        metsr_sim_dir=None,
        timeout=20,
        verbose=False,
        render=True,
        record="",
        run_name=None
    ):
        super().__init__()

        self.timestep = timestep
        self.sim_timestep = sim_timestep # This should represent the timestep recorded in the METSR config 
        self.map_path = map_path
        self.sim_ticks_per = int(round((timestep / sim_timestep)))
        assert math.isclose(self.sim_ticks_per, timestep / sim_timestep)

        self.bubble_size = bubble_size
        self.render= render
        self.record = record
        self.run_name = run_name
        self.metsr_sim_dir = metsr_sim_dir

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
                self.xml_to_xodr_map = _utils.generate_map(str(xml_map)) #convert pathlib obj to str for XML tree TODO what is best practice? 
                self.xml_to_xodr_intersections = _utils.generate_signal_map(str(xml_map))
            except Exception as e:
                raise RuntimeError(f"CARLA could not load world '{carla_map}'") from e
        else:
            #TODO figure out how to properly do the map handling here 
            if str(map_path).endswith(".xodr"):
                with open(map_path) as odr_file:
                    self.world = self.carla_client.generate_opendrive_world(odr_file.read())
            else:
                raise RuntimeError("CARLA only supports OpenDrive maps")

        if traffic_manager_port is None:
            traffic_manager_port = carla_port + 6000
            assert traffic_manager_port != metsr_port, f"Specified Traffic manager port {traffic_manager_port} is not available"
        self.tm = self.carla_client.get_trafficmanager(traffic_manager_port)
        self.tm.set_synchronous_mode(True)
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        print(f"Relaxed timestep restriction for testing?")
        assert sim_timestep <= .1 , f"timestep must be less that 0.1"
        settings.fixed_delta_seconds = sim_timestep 
        self.world.apply_settings(settings)
        verbosePrint("Map loaded in simulator.")

        # self.scenario_number = 0
        verbosePrint("Carla was initialized correctly proceeding to Metsr")

        # Setting up Metsr simulator 
        if self.metsr_sim_dir is not None:
            self.metsr_client = METSRClient(host=metsr_host,
                                            port=metsr_port,
                                            sim_folder=metsr_sim_dir, 
                                            verbose=verbose)
        else:
            self.metsr_client = METSRClient(host=metsr_host,
                                        port=metsr_port,
                                        verbose=verbose)


        verbosePrint("Clients have successfully been initialized")

        print(f"Creating CoSimulator with timestep: {self.timestep} and Ticks per step as: {self.sim_ticks_per}")

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
            timestep=self.timestep,
            sim_ticks_per=self.sim_ticks_per,
            tm=self.tm,
            bubble_size=self.bubble_size,
            render=self.render,
            record=self.record,
            mappings=self.xml_to_xodr_map,
            xml_to_xodr_intersections = self.xml_to_xodr_intersections,
            run_name= self.run_name,
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
    def __init__(self, scene, carla_client, metsr_client, timestep, sim_ticks_per, tm, render ,record, mappings, xml_to_xodr_intersections, bubble_size=100, run_name=None, **kwargs ):
    
        # Carla and metrs simulators
        self.carla_client = carla_client
        self.metsr_client = metsr_client
        self.timestep = timestep    # Timestep for each step 
        self.sim_ticks_per = sim_ticks_per
        
        # Initializing CARLA params
        self.tm = tm # Carla Traffic manager
        self.carla_world = self.carla_client.get_world()
        self.map = self.carla_world.get_map()
        self.blueprintLib = self.carla_world.get_blueprint_library()
        self.carla_cameraManager = None
        self.render = render
        self.record = record
        self.cameraManager = None

        # Initializing METSR params
        self.next_pv_id = 0
        self.pv_id_map = {}
        self.frozen_vehicles = set()
        self.scenic_to_metsr_map = mappings
        self._client_calls = []
        self.count = 0

        # CoSim related params
        self.bubble_size = bubble_size
        self.bubble_roads = [] 
        self.workspace = scene.workspace
        self.carla_control_roads = {}
        self.bubble_spawn_queue = set({})
        self.frozen_scenic_roads = []
        self.xml_to_xodr_intersections = xml_to_xodr_intersections
        self.metsr_actors = []
        self.carla_actors = []
        self.metsr_road_lines = []
        self.spawn_points = self.carla_world.get_map().get_spawn_points()
        self.completed_route = {}
        self.metsr_road_cache = {}
        self.road_pop_density = {}
        self.grp = None
        self.run_name = run_name

        # For tracking / data collection
        self.bubble_sizes = []
        self.total_active_vehicles = []


        super().__init__(scene, timestep=timestep, **kwargs)


    def setup(self) -> None:
        """
        Docstring for setup

        Setup the simulation instance 
        """
        # Updated version takes no arguements
        self.metsr_client.reset() 
        # Start the visualization once
        verbosePrint(f"Initializing METS-R visualization server")
        self.metsr_client.start_viz(server_port=8080)
        self.valid_metsr_roads = self.metsr_client.query_road()['id_list']

        self.network_helper = network_cache(self.workspace,
                                            self.scenic_to_metsr_map,
                                            self.valid_metsr_roads)
        
        world_map = self.carla_world.get_map()
        self.grp = GlobalRoutePlanner(world_map, sampling_resolution=2.0)

        weather = self.scene.params.get("weather")
        if weather is not None:
            if isinstance(weather, str):
                self.carla_world.set_weather(getattr(carla.WeatherParameters, weather))
            elif isinstance(weather, dict):
                self.carla_world.set_weather(carla.WeatherParameters(**weather))

        # Setup HUD
        # self.render=False
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
            print(f"starting recording")
            if not os.path.exists(self.record):
                os.mkdir(self.record)
            name = "{}/scenario{}.log".format(self.record, self.scenario_number)
            # Carla is looking for an absolute path, so convert it if necessary.
            name = os.path.abspath(name)
            self.carla_client.start_recorder(name)

        # Create objects.
        super().setup()
        
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

        self.carla_world.tick()  ## allowing manualgearshift to take effect  

        for obj in self.scene.objects:
            if obj.carla_actor_flag:
                if obj.speed is not None and obj.speed != 0:
                    raise RuntimeError(
                        f"object {obj} cannot have a nonzero initial speed "
                        "(this is not yet possible in CARLA)"
                    )

    def createObjectInMetsr(self, obj: Object, route_generation_attempts=100, origin: str = None) -> None:
        """
        Docstring for createObjectInMetsr
        
        :param obj: Cosimulation car object
        :type obj: Scenic Object

        Creates vehicle inside the METSR simulator
        """    
        dest = random.choice(self.spawn_points)
        pos = utils.carlaToScenicPosition(dest.location)
        for _ in range(route_generation_attempts): # max 20 tries
            route = self.metsr_client.query_route(obj.position.x, obj.position.y, pos.x, pos.y, transform_coords=True)["DATA"][0]
            if route != "KO":
                if len(route['road_list']) > 1:
                    break
       
        assert route != "KO", f"Failed to generate trajectory for obj in position (x,y) : {(obj.position.x, obj.position.y)}"
        
        obj.route = route['road_list']
        call_kwargs = {
            "vehID": self.getMetsrPrivateVehId(obj),
            "origin": obj.route[0],
            "destination": obj.route[-1],
        }

        self.metsr_client.generate_trip_between_roads(**call_kwargs)
        self.metsr_client.teleport_trace_replay_vehicle(vehID= self.getMetsrPrivateVehId(obj),
                                                        roadID=obj.route[0],
                                                        laneID=0,
                                                        x=obj.position.x, 
                                                        y=obj.position.y,  
                                                        private_veh = True, 
                                                        transform_coords=True)
        
   
        self.metsr_client.update_vehicle_route(self.getMetsrPrivateVehId(obj), route['road_list'], private_veh=True)


        
   
    def createObjectInCarla(self, obj: Object, update_orientation: bool = False, trajectory: list[carla.Transform] = None, metsr_data: dict = None) -> None:
        """
        Docstring for createObjectInCarla
        
        :param obj: Cosimulation car object
        :type obj: Scenic Object
        :param update_orientation: Flag to trigger adaptive spawn orientation according to object location
        :type update_orientation: bool

        """
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
            snapToGround=obj.snapToGround
        )
        if update_orientation:
            road = self._nearest_road(obj)
            rot = utils.scenicToCarlaRotation(road.orientation[obj.position])
        else: 
            rot = utils.scenicToCarlaRotation(obj.orientation)

        transform = carla.Transform(loc, rot)
        # Color, cannot be set for Pedestrians
        if blueprint.has_attribute("color") and obj.color is not None:
            c = obj.color
            c_str = f"{int(c.r*255)},{int(c.g*255)},{int(c.b*255)}"
            blueprint.set_attribute("color", c_str)
        try:
            carlaActor = self.carla_world.spawn_actor(blueprint, transform)
        except Exception as e:
            print(f"obj {obj.name} with snapToGround: {obj.snapToGround} and loc: {loc.x,loc.y,loc.z} with rot: {rot}")

            print(f"Failed to spawn actor in position: {obj.position}")
            print(f"Identifying cause of collision")
            _utils.within_threshold_to(obj, self.carla_actors, verbose=True)
            print(f"="*25)
            #Pass spawn errors for now
            raise SimulationCreationError(f"Unable to spawn object {obj} due to error: {e}")
            
        

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

            if trajectory != None:
                carlaActor.set_autopilot(True)
                self.tm.set_path(carlaActor, trajectory)

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
        
        obj.spawn_guard = 2
        obj.carla_actor_flag = True
        return True
   
    

    def createObjectInSimulator(self, obj: Object) -> None:
        """
        Docstring for createObjectInSimulator

        Spawn the object in the appropriate simulator
            (i) Ego is spawned in both simulators
            (ii)Ticks metsr to allow the vehicle to enter the road if the simulation has not started

        """
        assert obj.origin,      "All objects must have an origin"
        assert obj.destination, "All objects must have an destination"
        
        assert hasattr(obj, "carla_actor_flag"), "All objects must have attribute: carla_actor_flag"
        if obj == self.objects[0]: # Special handling for ego 
            self.ego = obj
            self.spawn_ego(obj)
            self.carla_actors.append(obj)
            self.metsr_actors.append(obj)
        else:
            self.createObjectInMetsr(obj)
            self.metsr_actors.append(obj)
            obj.finished_route = False # Track route completion for autopilot
            if self.count == 0:
                self.metsr_client.tick() # allow obj to enter road if possible
            obj.carla_actor_flag = False
            obj.spawn_guard = 0
        # Track autopilot behaviors TODO redundant?
        obj.active_autopilot = False
        obj.autopilot_action = False
        obj.trip_start = 0

    def spawn_ego(self,obj: Object) -> None:
        """
        docstring for spawn_ego

        :param ego: Simulation ego object
        :type ego: EgoCar 

        Special handling for spawning the Ego vehicle
            (1) First spawn ego in METSR on the appropriate lane (set by Scenic)
            (2) Teleport ego to the precise spawn location in MESTR
            (3) Collect exact spawn location to define bubbble region
            (4) Freeze CoSimulated regions inside METSR
            (5) Spawn ego inside CARLA 
        """
        obj.bubble = CircularRegion(center=[obj.position.x,
                                            obj.position.y],
                                            radius=self.bubble_size)

        bubble_roads = self._get_bubble_roads()
        new_roads, _ = self.classify_bubble_roads(bubble_roads)
        self.freeze_roads(new_roads) # Freeze lanes according to Ego Spawn
        self.metsr_client.tick()

        self.createObjectInMetsr(obj) # Set the METSR vehicle origin to match ego spawn
        self.metsr_client.tick() # Allow the vehicle to spawn
                           
        spawn_success = self.createObjectInCarla(obj) # spawn ego in updated location and update orientation
        assert spawn_success, f"Invalid spawn selection point at : {obj.position}" 

    def getCarlaProperties(self, obj : Object, properties : dict) -> dict[str, float | Vector | int]:
        """
        Docstring for getCarlaProperties

        :param obj: Cosimulation car object
        :type obj: Scenic Object
        
        return objects properties from the Carla simulator
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
        return values


    def getMetsrProperties(self, obj: object, properties : dict) -> dict[str, float | Vector | int]:
        """
        Docstring for getMetsrProperties

        :param obj: Cosimulation car object
        :type obj: Scenic Object
        
        return objects properties from the METSR simulator
        """
        raw_data = self.obj_data_cache[obj]
        # check vehicle state
        is_frozen = "roadID" not in raw_data
        if obj in self.frozen_vehicles and is_frozen:
            return None # skip update if frozen for more than 1 step

        if is_frozen: # update froozen vehicles
            self.frozen_vehicles.add(obj)
        else:
            if obj in self.frozen_vehicles:
                self.frozen_vehicles.remove(obj)

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

    def getProperties(self, obj : Object, properties : dict)-> dict[str, float | Vector | int]:
        """
        Docstring for getProperties

        :param obj: Cosimulation car object
        :type obj: Scenic Object
        
        return objects properties for any CoSim object
        """
        assert hasattr(obj, "carla_actor_flag"), f"Object is not assigned properly to a simulator instance"
        if obj.carla_actor_flag: 
            properties = self.getCarlaProperties(obj,properties)
        else:
            properties =  self.getMetsrProperties(obj,properties)
        return properties

    def getMetsrPrivateVehId(self, obj: Object) -> int:
        """
        Docstring for getMetsrPrivateVehId

        :param obj: Cosimulation car object
        :type obj: Scenic Object

        Return unique vehicle idea
            Generates a new ID if none exists for vehicle
        """
        if obj not in self.pv_id_map:
            self.pv_id_map[obj] = self.next_pv_id
            self.next_pv_id += 1
        return self.pv_id_map[obj]

    def tick_carla(self) -> None:
        """
        Docstring for tick_carla

        Tick Carla client for a single step
        """
        for _ in range(self.sim_ticks_per):
            self.carla_world.tick()
  
    
    def tick_metsr(self) -> None:
        """
        Docstring for tick_metsr

        Tick Metsr client for a single step
        """
        for _ in range(self.sim_ticks_per):
            self.metsr_client.tick()

    def step(self) -> None:
        """
        Docstring for step

        Step both simulators:
            (1): Update the high fidelity region based on the ego's new locatin
            (2): Spawn and destroy objects according to region changes
            (3): Tick both clients and synchronize states
            (4): Compute new bubble region 
        """
        self.road_pop_density = {road: 0 for road in self.valid_metsr_roads}

        # (1): Update the high fidelity region based on the ego's new locatin
        bubble_roads = self._get_bubble_roads()
        new_roads, old_roads = self.classify_bubble_roads(bubble_roads)
        self.release_roads(old_roads)
        self.freeze_roads(new_roads)
        intersections = self.get_bubble_intersections(bubble_roads=bubble_roads, bubble_region=self.ego.bubble)
     
        # (2): Spawn and destroy objects according to region changes
        bubble_road_ids = [road.id for road in bubble_roads]
        intersection_ids = [intersection.id for intersection in intersections]
        self.update_bubble_objects(bubble_road_ids, intersection_ids)

        # (3): Tick both clients and synchronize states
        self.tick_carla()
        self.synchronize_clients()
        self.tick_metsr()

        if self.render:
            self.cameraManager.render(self.display)
            pygame.display.flip()
            self.metsr_client.render()
       
        self.bubble_sizes.append(len(self.carla_actors))
        self.total_active_vehicles.append(len(self.objects) - (len(self.frozen_vehicles) + len(self.bubble_spawn_queue)))
        if self.count % 200 == 0: 
            print(f"Step: {self.count}. Total actors: {len(self.objects)}, bubble queue:{len(self.bubble_spawn_queue)} ")
            print(f"Total active vehicles: {self.total_active_vehicles[-1]}, frozen vehicles {len(self.frozen_vehicles)}")
            print(f"Total bubble actors: {len(self.carla_actors) + len(self.bubble_spawn_queue)}")
            print(f"Completed routes: {len(list(self.completed_route.keys()))}")
            print(f"Road densities: {self.road_pop_density}")
        
        if self.count % 50 == 0:
            out_file = self.run_name + "_veh_data.csv"

            data_dict_at_i = {
                "active_vehicles": self.total_active_vehicles[-1],
                "bubble_actors" : len(self.carla_actors), 
                "bubble_queue": len(self.bubble_spawn_queue),
                "completed_routes": len(list(self.completed_route.keys())),
                **self.road_pop_density
            }
            final_df = pd.DataFrame([data_dict_at_i])
            del data_dict_at_i

            if self.count == 0:
                os.makedirs(os.path.dirname(out_file), exist_ok=True)
                final_df.to_csv(out_file,
                                mode="w",
                                header=True)
            else:
                final_df.to_csv(out_file,
                                mode="a",
                                header=False)
            
        self.count += 1
        # (4): Compute new bubble region and process behavior interrupts
        self.ego.bubble = CircularRegion(center=[self.objects[0].x, self.objects[0].y], radius=self.bubble_size)

        
    
    def get_bubble_intersections(self, bubble_roads: list[Road], bubble_region: CircularRegion) -> list[Intersection]:
        """
        Collect any intersections that are either
            (1) Intersecting the CoSim bubble
            (2) Are connected to the bubble via AT LEAST 2 roads
   
        :return: The set of all intersections meeting the above criteria
        :rtype: list[Intersection]
        """
        bubble_intersections = []
        for intersection in self.network_helper.network_intersections:
            if intersection.intersects(bubble_region):
                bubble_intersections.append(intersection)
                continue
            intersection_roads = intersection.roads
            count = 0
            for road in intersection_roads:
                if road in bubble_roads:
                    count += 1
                if count > 1:
                    bubble_intersections.append(intersection)
                    break
        return bubble_intersections
    
    def initiate_autopilot(self, obj : Object) -> bool:
        """
        docstring for initiate_autopilot

        :param obj: Target vehicle to initiate autopilot for

        Activates autopilot for a simulation vehicle
         (i) If the vehicle is in Carla, queries metsr for the target trajectory
             then converts that to a corresponding set of Carla Waypoints for Carla autopilot
         (2) If the vehicle is in METSR updates the overwrite flag for manually controlling the vehicle        
        """
        if obj.carla_actor_flag:
            obj.carlaActor.set_autopilot(True) # Set the autopilot with no specific trajectory if none is found?
            success = True
        else:
            obj.override_autopilot = False
            success = True
        
        return success
    

    def synchronize_clients(self, obj: Object | list[Object] = None):
        """
        Docstring for synchronize_clients
        
        :param obj: Cosimulation car object
        :type obj: Scenic Object[s]

        Default : updates all CoSimulated object states in the METSR simulator
            (1) Can choose to specify which objects should be updated with obj arguement
        """
        if obj != None and not isinstance(obj, list):
            carla_actors = [obj]
        elif obj != None and isinstance(obj, list):
            carla_actors = obj
        else:
            carla_actors = self.carla_actors
        
        all_veh_data = self._collect_metsr_vehicle_data(carla_actors)

        for obj in carla_actors: # TODO clean up all this special handling as some of these cases are unneccesary
            try: 
                loc = obj.carlaActor.get_location()
            except Exception as e:
                print(f"Caught error {e}")
                warnings.warn(f"Object {obj.name if hasattr(obj,'name') else obj} automatically removed by CARLA likely due to deadlock")
                self.remove_bubble_object(obj,destroy=False)
                print(f"Deadlocked vehicle: {obj.name} was removed")
                continue
                # raise ObjectMissingInSimulation(f"Unable to access object in simulation, if this issue persists try removing CARLA autopilot")
           
            if (loc.x,loc.y,loc.z) == (0,0,0): # Carla object still in the process of processing obj spawn
                continue
            
            veh_data = all_veh_data[obj]
            vehID = self.getMetsrPrivateVehId(obj)
            lane = self.network_helper._nearest_lane(obj)
           
            road_id = None
            if lane: 
                key = f"{lane.road.id}_{lane.id}"
                if key in self.network_helper.scenic_to_metsr_map_lanes:
                    roads = self.network_helper.scenic_to_metsr_map_lanes[key]
                    if len(roads) > 1:
                        metsr_road = self.identify_nearest_road(obj, roads)
                    else:
                        metsr_road = roads[0].split("_")[0]
                    if metsr_road in self.network_helper.metsr_represented_roads:
                        road_id = metsr_road
                if veh_data["roadID"] != road_id and road_id is not None: # Entering new road within metsr network\
                    self.metsr_client.enter_next_road(vehID, roadID=road_id, private_veh = True)
                    if road_id == veh_data["destRoadID"]:
                        self.completed_route[obj] = True
                        obj.finished_route = self.count
          
            bearing = _utils.get_metsr_rotation(obj.carlaActor.get_transform().rotation.yaw)
            # Check if objects are desynchronized
            if not math.isclose(loc.x, veh_data['x']) or not math.isclose(-loc.y, veh_data['y']):
                self.metsr_client.teleport_cosim_vehicle(vehID, loc.x, -loc.y, bearing=bearing, private_veh = True, transform_coords = True)

    def identify_nearest_road(self, obj: Object, roads: list[str] ) -> str:
        """
            Docstring for identify_nearest_road

            :param obj: vehicle to identify road for 
            :rtype obj:  Object
            :param roads: List of roads which obj's curr lane maps to
            :rtype roads: List[str]

            Given a non-unique mapping between road formats selected the target road based on object distance to the start 
            lane
        
        """
        roads = [road_lane.split("_")[0] for road_lane in roads]
        best_road = None
        best_dist = math.inf
        for road in roads:
            if road in self.network_helper.intersection_road_links:
                continue
            elif road not in self.metsr_road_cache:
                start = self.metsr_client.query_centerline(road,lane_index=0,transform_coords=True)["DATA"][0]["centerline"][0]
                self.metsr_road_cache[road] = start
            else:
                start = self.metsr_road_cache[road]
            
            dist = math.dist(start[:2], obj.position[:2])
            if dist < best_dist:
                best_dist = dist
                best_road = road
        
        return best_road



    def classify_bubble_roads(self, bubble_roads : list[Road]) -> tuple[list[str], list[str]]:
        """
        Docstring for update_carla_roads
    
        :param bubble_regions: List of objects with a designated "bubble region" constituting the CoSim region
        :type obj: List[Object] or None

        Collects all roads which are intersecting the bubble region 
            (1) Default region is defined by the ego the region can be updated by passing objects with their corresponding regions
        """
        road_ids = []
        bubble_road_ids = []
        for road in bubble_roads:
            r_id = str(road.id)
            if r_id not in self.network_helper.scenic_unique_roads:
                road_ids.append(r_id)
                bubble_road_ids += self.map_scenic_to_metsr_road(road)
        bubble_roads += self.network_helper.repair_mapping(road_ids, bubble_road_ids)
        self.bubble_roads = bubble_roads # List of roads contained in the CoSim region

        self.frozen_roads = list(self.carla_control_roads.keys())
        # Collect roads into new and old for freeze/unfreezing 
        new_roads = [id for id in bubble_road_ids if id not in self.frozen_roads]
        old_roads = [id for id in self.frozen_roads if id not in bubble_road_ids]
        return new_roads, old_roads
    
    
   
    def update_bubble_objects(self, bubble_roads: list[Road], bubble_intersections: list[Intersection]) -> None: 
        """
        Docstring for update_bubble_objects
        
        :param bubble_roads: a list of Scenic lanes which constitute the cosimulation region 
        :type bubble_roads: list[Road]
        :param intersections: a list of Scenic intersections which are contained or touching the cosimulated region
                              (A lane must either intersect or have to connecting roads in the cosimulation region)
        :type intersections: list[intersection]

        (1) Remove all objects from CARLA which have either
            (i) finished their associated route
            (ii) left the region
        (2) Spawn new objects in the Cosimulation region if
            (i)   Their is enough room in the obj's current location to spawn
            (ii)  The vehicle is not currently waiting to spawn in a metsr queue
                
        """
        all_veh_data = self._collect_metsr_vehicle_data(self.objects[1:])
        for obj in self.objects[1:]: 
            veh_data = all_veh_data[obj]
            obj.spawn_guard = max(0, obj.spawn_guard - self.sim_ticks_per) # Total client ticks, always > 0

            # Skip vehicles which have not entered the roadway or have completed their route 
            if ('roadID' not in veh_data) or obj in self.completed_route:
                if obj.carla_actor_flag:
                    self.remove_bubble_object(obj)
                else:
                    if obj not in self.completed_route:
                        self.completed_route[obj] = True
                        obj.finished_route = self.count
                continue
            else:
                if obj not in self.bubble_spawn_queue:  
                    self.road_pop_density[veh_data['roadID']] += 1

            outside_bubble = False
            road = self.network_helper._nearest_road(obj)
            id = road.id if road else None
            intersection = None

            if id not in bubble_roads:
                intersection = self.network_helper._get_intersection(obj, road)
                if intersection not in bubble_intersections:
                    outside_bubble = True
                              
            # Remove vehicles which have left the cosimulation region and spawn vehicles which have entered
            if outside_bubble:
                if obj.carla_actor_flag:
                    if obj.spawn_guard == 0:
                        self.remove_bubble_object(obj)
                else:
                    if obj in self.bubble_spawn_queue:
                        self.bubble_spawn_queue.remove(obj)


            else:  # inside bubble
                if not obj.carla_actor_flag:    # Vehicle needs to be spawned in CoSim region
                    not_enough_space = _utils.within_threshold_to(obj, self.carla_actors, verbose=False)
                    if not_enough_space:        # ensure there is sufficient room before spawning
                        if obj not in self.bubble_spawn_queue:
                            self.bubble_spawn_queue.add(obj)
                        continue
                    else: 
                        self.createObjectInCarla(obj, update_orientation=True) # Spawn Vehicle        
                        if obj in self.bubble_spawn_queue:
                            self.bubble_spawn_queue.remove(obj)
                    
                        self.carla_actors.append(obj)
                        self.metsr_actors.remove(obj)
            
            
 
    def freeze_roads(self, keys: list[str]) -> None:
        """
        Docstring for freeze_roads
        
        :param keys: RoadIDs for METSR indexed roads
        :type keys: list[str]
        
        Query Metsr to freeze simulation and control of given lanes
        """
        keys = set(keys)
        for key in keys:
            assert key not in self.carla_control_roads, "Attempted to freeze already frozen lane"
            if key not in self.network_helper.intersection_road_links: # Skip roads not recognized by metsr
                self.carla_control_roads[key] = True    # Keep track of frozen lanes
                self.metsr_client.set_cosim_road(key)
            

    def release_roads(self,keys: list[str]) -> None:
        """
        Docstring for release_roads
        
        :param keys: RoadIDs for METSR indexed roads
        :type keys: list[str]
        
        Query Metsr to begin re-simulating and control given lanes
        """
        keys = set(keys) 
        for key in keys:
            assert key in self.carla_control_roads, "Attempted to release non frozen lane"
            if key not in self.network_helper.intersection_road_links: # Skip roads not recognized by metsr
                del self.carla_control_roads[key] # Remove frozen lane from record
                self.metsr_client.release_cosim_road(key)

    def destroy_carla_obj(self,obj) -> None:
        """
        Docstring for destroy_carla_obj

        Destroys obj from CARLA simulation
        
        :param obj: Carla object to be destroyed
        """
        if obj.carlaActor is not None:
            if isinstance(obj.carlaActor, carla.Vehicle):
                obj.carlaActor.set_autopilot(False, self.tm.get_port())
            if isinstance(obj.carlaActor, carla.Walker):
                obj.cralaController.stop()
                obj.carlaController.destroy()
            obj.carlaActor.destroy()
            obj.carlaActor = None # Set this to None to prevent reaccess of a previously deleted vehicle? 
    
    def remove_bubble_object(self,obj, destroy=True) -> None:
        """
        Docstring for remove_bubble_object

        :param obj: object to be deleted
        :type obj: Car
        """
        if obj.autopilot_action and obj.active_autopilot:
            obj.active_autopilot = not(_utils.disable_carla_autopilot(obj, self.tm))
            obj.trajectory = None   
        if destroy:
            self.destroy_carla_obj(obj)
        obj.carla_actor_flag = False
        self.carla_actors.remove(obj)
        self.metsr_actors.append(obj)


    def destroy(self) -> None:
        """
        Docstring for destroy
        
        Destroy both simulators instances i.e (METSR, CARLA)
        """
        verbosePrint(f"Closing METS-R visualization server")
        self.metsr_client.stop_viz()

        print(f"Logging trip times")
        print(f"="*25)
        self._log_trip_times()
        print(f"="*25)

        # METSR destroy
        if self.metsr_client.verbose:
            print("Client Messages Log:")
            print("[")
            for call in self.client._messagesLog:
                print(f"    {call},")
            print("]")

        # "CARLA destroy"
        for obj in self.carla_actors:
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

        super().destroy()

    def map_scenic_to_metsr_road(self, road: Road) -> list[str]:
        """Maps Scenic road to equvialent METSR roads, 1->M mapping"""
        return self.network_helper.map_scenic_to_metsr_road(road)
    
    def map_scenic_to_metsr_lanes(self, lane: Lane) -> set[str]:
        """Map Scenic lane to equivalent METSR road, guareneteed 1->1 mapping"""
        return self.network_helper.map_scenic_to_metsr_lanes(lane)
    
    def generate_metsr_trajectory(self, trajectory: list[Lane]) -> list[str] | None:
        """Convert a scenic specified trajectory to an equivalent metsr route"""
        return self.network_helper.generate_metsr_trajctory(trajectory)

    def _nearest_road(self, obj: Object, allow_offroad: bool = True, radius_size: int = 30) -> tuple[Road, str]:
        """Collect the nearest road to obj location"""
        return self.network_helper._nearest_road(obj, allow_offroad, radius_size)

    def _nearest_lane(self,obj : Object, allow_offlane : bool = True, radius_size : int = 50, allow_intersection_links : bool = True) -> Lane: 
        """Collect the nearest lane to obj location"""
        return self.network_helper._nearest_lane(obj, allow_offlane, radius_size, allow_intersection_links)
    
    def _get_intersection(self, obj: Object, road: Road ) -> Intersection | None:
        """Returns the intersection the obj is on if any"""
        return self.network_helper._get_intersection(obj, road)
   
    def _get_bubble_roads(self, bubble_region: CircularRegion | None = None) -> list[Lane]:
        """Collect all roads which overlap the designated bubble region"""
        if bubble_region == None: # Default is attached to ego
            bubble_region = self.ego.bubble
        else:
            bubble_region = bubble_region # User specified (for added functionality later)
        return self.network_helper._get_bubble_roads(bubble_region)

    
    def executeActions(self, allActions) -> None:
        """
        Docstring for executeActions
        
        Apply control updates which were accumulated while executing the actions
        Filters out actions for Carla only objects
        
        :param allActions: ?
        """
        carla_actions = {}
        for obj in self.agents:
            carla_actions[obj] = allActions[obj] 
        super().executeActions(carla_actions)
        for obj in self.agents:
            if obj.carla_actor_flag: # Processing CARLA actors
                if not obj.autopilot_action and obj.active_autopilot: # Disable autopilot first to enable smooth transitions
                    obj.active_autopilot = not(_utils.disable_carla_autopilot(obj, self.tm))
                elif obj.autopilot_action and not obj.active_autopilot:
                    if not hasattr(obj, "trajectory"):
                        obj.active_autopilot = self.initiate_autopilot(obj)
                        obj._control = None # TODO What does this do? 
                    elif obj.trajectory is None:
                        obj.trajectory = self.metsr_trajectory_to_carla(obj)
                        self.tm.set_path(obj.carlaActor, obj.trajectory)
                        self.initiate_autopilot(obj)
                        obj.active_autopilot = True 
                        obj.autopilot_action = True
                    else:
                        self.tm.set_path(obj.carlaActor, obj.trajectory)
                        self.initiate_autopilot(obj) 
                        obj.active_autopilot = True
                        obj.autopilot_action = True             
                else:
                    ctrl = obj._control
                    if ctrl is not None:
                        obj.carlaActor.apply_control(ctrl)
                        obj._control = None
            else:
                if not obj.autopilot_action: # Default is autopilot 
                    target_acc = obj.target_acceleration if hasattr(obj, "target_accleration") else 0 # apply, if no action is taken no movement
                    self.metsr_client.control_vehicle(self.getMetsrPrivateVehId(obj), target_acc, private_veh=True)
                if hasattr(obj, "trajectory"):
                    if obj.trajectory and not obj.autopilot_action:
                        if not self.trajectory_is_active:
                            metsr_trajectory = self.generate_metsr_trajectory(obj.trajectory)
                            if metsr_trajectory:
                                self.metsr_client.update_vehicle_route(self.getMetsrPrivateVehId(obj), metsr_trajectory, private_veh=True)
                                self.trajectory_is_active = True


    def metsr_trajectory_to_carla(self, obj : object) -> list[Lane]:
        """
        docstring for metsr_tractory_to_carla

        :param obj: Target object to generate trajectory for
        :typ obj: Object

        Convert proposed METS-R trajectory to a sequence of equivalent Lane's
        """
        cosim_data = self.metsr_client.query_coSimVehicle()
        trajectory = None
        if obj.carla_actor_flag:
            VehID = self.getMetsrPrivateVehId(obj) 
            for data_entry in cosim_data['DATA']:
                if data_entry['ID'] == VehID:
                    route_data = data_entry['route']
                    trajectory = self.generate_carla_trajectory(route = route_data, obj=obj)
        
        if trajectory is None:
            route = obj.route
            trajectory = self.generate_carla_trajectory(route=route, obj=obj)
        
        return trajectory
    
    def generate_carla_trajectory(self, route: list[str], obj: Object) -> list[Lane]:
        """
         Docstring for generate_carla_trajaectory

         Given a sequence of SUMO roads generate a sequence of CARLA waypoints 
         for CARLA autopilot to follow 

         :param route: List of SUMO road IDs
         :type route: List[str]
        """
        end = route[-1]
        target_end = self.metsr_client.query_centerline(end,lane_index=0,transform_coords=True)["DATA"][0]["centerline"][1]

        target_start = carla.Location(obj.position.x, -obj.position.y, 0) # convert METS-R points to CARLA
        target_end = carla.Location(target_end[0], -target_end[1], 0)
        
        route = self.grp.trace_route(target_start, target_end)

        locations = [wp.transform.location for wp, _ in route]
        return locations
                    
    def updateObjects(self) -> None:
        """
        Docstring for updateObjects
        
        Update object properties for METSR simulated objects
        """
        self.obj_data_cache = self._collect_metsr_vehicle_data(self.metsr_actors)
        super().updateObjects()
        self.obj_data_cache = None 

    def _collect_metsr_vehicle_data(self, objects: list[Object] | None = None):
        """
        Docstring for _collect_metsr_vehicle_data

        :param objects: List of objects which data should be queried
        :rtype objects: Scenic vehicle object

        Query metsr for state infomation on vehicle's default is all vehicles
        """
        if objects is None: # all objects by defualt
            obj_veh_ids = [self.getMetsrPrivateVehId(obj) for obj in self.objects]
            raw_veh_data = self.metsr_client.query_vehicle(obj_veh_ids, True, True)
            all_veh_data = {obj: raw_veh_data['DATA'][i] for i, obj in enumerate(self.objects)}
        else:           
            obj_veh_ids = [self.getMetsrPrivateVehId(obj) for obj in objects]
            raw_veh_data = self.metsr_client.query_vehicle(obj_veh_ids, True, True)
            all_veh_data = {obj: raw_veh_data['DATA'][i] for i, obj in enumerate(objects)}
        return all_veh_data
    

    def _save_metsr_state(self, file_name=None) -> None:
        """
        docstring for _save_metsr_state

        Saves metsr state to a file to allow for reproducible replay
        """
        if file_name == None:
            save_file = f"metsr_state_at_{self.count}.bin"
        else:
            save_file = file_name
        self.metsr_client.save(save_file)



    def _log_trip_times(self, file_name=None):
        """
            docstring for _log_trip_times
            
            :param file_name: target location and name for logs
            :rtype file_name: str

            Generate csv file containing total time to route completion for each vehicle 
        """
        out_file = file_name if file_name else f"{self.run_name}_trip_logs.csv"
        trip_dict = {obj.name: obj.finished_route  - obj.trip_start  if hasattr(obj, "finished_route") else None for obj in self.objects[1:]}
        trip_df = pd.DataFrame([trip_dict])

        os.makedirs(os.path.dirname(out_file), exist_ok=True)
        trip_df.to_csv(out_file)
