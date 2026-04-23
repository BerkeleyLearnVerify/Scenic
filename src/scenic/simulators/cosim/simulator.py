from scenic.core.simulators import Simulation, Simulator
from scenic.core.vectors import Orientation, Vector
from scenic.syntax.veneer import verbosePrint
from scenic.simulators.metsr.client import METSRClient
from scenic.simulators.cosim.utils.utils import *
from scenic.core.regions import CircularRegion
from scenic.core.object_types import Object
from scenic.core.simulators import SimulationCreationError
from scenic.domains.driving.roads import Lane, Intersection, Road
from scenic.domains.driving.simulators import DrivingSimulation, DrivingSimulator

import pygame
import warnings
import os
import math
import scenic.simulators.cosim.utils.utils as _utils
import scenic.simulators.carla.utils.utils as utils


import scenic.simulators.carla.utils.visuals as visuals
from scenic.simulators.carla.blueprints import oldBlueprintNames
import random

try:
    import carla
except ImportError as e:
    raise ModuleNotFoundError('CARLA scenarios require the "carla" Python package') from e

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
        timestep=0.1, # Not entirely sure what the distinction between timestep and sim_timestep is in metsr
        sim_timestep=0.1,
        traffic_manager_port=None,
        timeout=20,
        verbose=False,
        render=True,
        record=""
    ):
        super().__init__()


        self.metsr_map_name = metsr_map
        self.timestep = timestep
        self.sim_timestep = sim_timestep # This should represent the timestep recorded in the METSR config 
        self.map_path = map_path
        self.sim_ticks_per = int(round((timestep / sim_timestep)))
        assert math.isclose(self.sim_ticks_per, timestep / sim_timestep)

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

        # self.scenario_numver = 0
        verbosePrint("Carla was initialized correctly proceeding to Metsr")

        # Setting up Metsr simulator 

        self.metsr_client = METSRClient(host=metsr_host, port=metsr_port, verbose=verbose)

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
    def __init__(self, scene, carla_client, metsr_client, timestep, sim_ticks_per, tm, render ,record, mappings, xml_to_xodr_intersections, bubble_size=100,  **kwargs ):
    
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
        self.workspace = scene.workspace
        self.carla_control_roads = {}
        self.bubble_spawn_queue = set({})
        self.frozen_scenic_roads = []
        self.xml_to_xodr_intersections = xml_to_xodr_intersections
        self.metsr_actors = []
        self.carla_actors = []

        # For tracking / data collection
        self.bubble_sizes = []
        self.total_active_vehicles = []
        self.routes_completed = 0


        super().__init__(scene, timestep=timestep, **kwargs)


    def setup(self) -> None:
        """
        Docstring for setup

        Setup the simulation instance 
        """
        # Updated version takes no arguements
        self.metsr_client.reset() 
        valid_metsr_roads = self.metsr_client.query_road()

        self.network_helper = network_cache(self.workspace,
                                            self.scenic_to_metsr_map,
                                            valid_metsr_roads)

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
                
        # TODO Waiting for map update
        # self._synchronize_signals()


    def createObjectInMetsr(self, obj: Object, origin: int = None, destination: int = None) -> None:
        """
        Docstring for createObjectInMetsr
        
        :param obj: Cosimulation car object
        :type obj: Scenic Object

        Creates vehicle inside the METSR simulator
        """
        assert obj.origin,  "Metsr objects must have a defined origin"
        assert obj.destination, "Metsr objects must have a defined destination"

        obj_origin = origin if origin else obj.origin
        obj_destination = destination if destination else obj.destination

        call_kwargs = {
            "vehID": self.getMetsrPrivateVehId(obj),
            "origin": obj_origin,
            "destination": obj_destination,
        }
        if origin or destination:
            self.metsr_client.generate_trip_between_roads(**call_kwargs)
        else:
            self.metsr_client.generate_trip(**call_kwargs)        


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
            lane = self._nearest_lane(obj)
            rot = utils.scenicToCarlaRotation(lane.orientation[obj.position])
        else: 
            rot = utils.scenicToCarlaRotation(obj.orientation)

        transform = carla.Transform(loc, rot)
        # print(f"Attempting to spawn obj {obj.name} in location: {transform.location}")
        # Color, cannot be set for Pedestrians
        if blueprint.has_attribute("color") and obj.color is not None:
            c = obj.color
            c_str = f"{int(c.r*255)},{int(c.g*255)},{int(c.b*255)}"
            blueprint.set_attribute("color", c_str)
        try:
            carlaActor = self.carla_world.spawn_actor(blueprint, transform)
        except Exception as e:
            return False
        
        if carlaActor is None:
            raise SimulationCreationError(f"Unable to spawn object {obj}")
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
            obj.finished_route = False
            if self.count == 0:
                self.metsr_client.tick()
            obj.carla_actor_flag = False
            obj.spawn_guard = 0

    def spawn_ego(self,obj: Object) -> None:
        """
        docstring for spawn_ego

        :param ego: Simulation ego object
        :type ego: EgoCar 

        Special handling for spawning the Ego vehicle
            (1) First spawn ego in METSR on the appropriate lane (set by Scenic)
            (2) Collect exact spawn location to define bubbble region
            (3) Freeze CoSimulated regions inside METSR
            (4) Set the appropriate Ego behavior
                (i) Predefined trajectory
                (ii)User defined behavior
                (iii) Default CARLA autopilot over METSR proposed route
            (5) Spawn ego inside CARLA with updated loc and trajectory
        """
        trajectory = None
        lane = self._nearest_lane(obj, allow_intersection_links = False)
        origin_str = self.map_scenic_to_metsr_lanes(lane)
      
        self.createObjectInMetsr(obj,origin=origin_str) # Set the METSR vehicle origin to match ego spawn
        self.metsr_client.tick() # Allow the vehicle to spawn
        car_data = self.metsr_client.query_vehicle(self.getMetsrPrivateVehId(obj), True, True)["DATA"][0]
        obj.position = Vector(car_data["x"], car_data["y"], 0)
        obj.final_road = None  

        obj.bubble = CircularRegion(center=[car_data["x"],
                                            car_data["y"]],
                                            radius=self.bubble_size)

        bubble_roads = self._get_bubble_roads()
        _, new_roads, _ = self.classify_bubble_roads(bubble_roads)
        self.freeze_roads(new_roads) # Freeze lanes according to Ego Spawn
        self.metsr_client.tick()
        car_data = self.metsr_client.query_vehicle(self.getMetsrPrivateVehId(obj), True, True)["DATA"][0]
       
        if hasattr(obj, "trajectory"): 
            if obj.trajectory is not None:
                trajectory = self.scenic_trajectory_to_carla(trajectory)

            elif hasattr(obj, "behavior"): # Default to CARLA autopilot behavior from spawn to random target road
                if obj.behavior is None:
                    for _ in range(len(self.network_helper.network_lanes)):
                        choice = random.choice(self.network_helper.network_lanes)
                        dest_roadID = self.map_scenic_to_metsr_lanes(choice)
                        if dest_roadID not in self.network_helper.intersection_road_links and dest_roadID != origin_str:
                            break
                    
                    trajectory_query = self.metsr_client.query_route_between_roads(origin_str,dest_roadID)
                    if "DATA" in trajectory_query:
                        trajectory = trajectory_query["DATA"][0]
                    
                    scenic_trajectory = self.generate_scenic_trajectory(lane, trajectory["road_list"])
                    if scenic_trajectory is not None:
                        trajectory = self.scenic_trajectory_to_carla(scenic_trajectory)          
            
        self.createObjectInCarla(obj, update_orientation=True, trajectory=trajectory) # spawn ego in updated location and update orientation
           

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
        is_frozen = "road" not in raw_data
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
            (4): Compute new bubble region and process behavior interrupts TODO
        """

        # (1)
        bubble_roads = self._get_bubble_roads()
        roads, new_roads, old_roads = self.classify_bubble_roads(bubble_roads)
        self.release_roads(old_roads)
        self.freeze_roads(new_roads)
        intersections = self.get_bubble_intersections(bubble_roads=roads, bubble_region=self.ego.bubble)
     
        # (2)
        self.update_bubble_objects(roads,intersections)

        # (3)
        self.tick_carla()
        self.synchronize_clients()

        if self.render:
            self.cameraManager.render(self.display)
            pygame.display.flip()
       
    
        self.bubble_sizes.append(len(self.carla_actors))
        self.total_active_vehicles.append(len(self.objects) - (len(self.frozen_vehicles) + len(self.bubble_spawn_queue)))
        self.count += 1
        if self.count % 100 == 0: 
            print(f"Step: {self.count}. Total actors: {len(self.objects)}, bubble queue:{len(self.bubble_spawn_queue)} ")
            print(f"Total active vehicles: {self.total_active_vehicles[-1]}, Routes completed: {self.routes_completed}, frozen vehicles {len(self.frozen_vehicles)}")
        self.tick_metsr()
       
        # (4)
        self.ego.bubble = CircularRegion(center=[self.objects[0].x,
                                                        self.objects[0].y],
                                                        radius=self.bubble_size)
        if self.ego.interrupt: #TODO support behaviors for objects when inside the bubble
            _utils.disable_carla_autopilot(self.ego)
        
    
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


    def _synchronize_signals(self) -> None:
        """
        docstring for _synchronize_signals
   
        Synchronize all lights from each map representation to the same timing schedule and state
            : has the side effect of resetting all lights to the start of their respective schedules
        """
        signals_ids = self.metsr_client.query_signal()['id_list']
        signal_data = self.metsr_client.query_signal(signals_ids)
        
        carla_world = self.carla_client.get_world()
        carla_traffic_lights = carla_world.get_actors().filter('traffic.traffic_light*')

        lights_by_opendrive_id = {light.get_opendrive_id(): light for light in carla_traffic_lights}
        updated_ids = {}

        for light_data in signal_data["DATA"]:
            light_id = light_data["groupID"]
            
            if light_id in self.xml_to_xodr_intersections:
                light_opendrive_ids = self.xml_to_xodr_intersections[light_id]

                light_config = self.get_light_config(light_data)
                
                if len(light_opendrive_ids) > 1:  
                    for open_drive_id in light_opendrive_ids:
                        if open_drive_id not in updated_ids: 
                            if open_drive_id in lights_by_opendrive_id:
                                light = lights_by_opendrive_id[open_drive_id]
                                self._update_carla_light_state(light, light_config)
                                updated_ids[open_drive_id] = True
                        else:
                            if open_drive_id in lights_by_opendrive_id:
                                light = lights_by_opendrive_id[open_drive_id]
                                # self._check_light_consistency(light, light_config)
                                self.metsr_client.update_signal(light_data['ID'], targetPhase=light_data['state'])

                else:
                    open_drive_id = light_opendrive_ids[0]
                    if open_drive_id not in updated_ids:
                        if open_drive_id in light_opendrive_ids:
                            light = lights_by_opendrive_id[open_drive_id]
                            self._update_carla_light_state(light, light_config)
                            updated_ids[open_drive_id] = True
                        else:
                            if open_drive_id in lights_by_opendrive_id:
                                light = lights_by_opendrive_id[open_drive_id]
                                # self._check_light_consistency(light, light_config)
                                self.metsr_client.update_signal(light_data['ID'], targetPhase=light_data['state'])

            else:
                assert True, f"Failed to find corresponding intersection for METSR light: {light_id}"
            
            self.metsr_client.tick()
            self.carla_world.tick() 

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
        
        all_veh_data = self._collect_metsr_vehicle_data(self.carla_actors)

        for obj in carla_actors: # TODO clean up all this special handling as some of these cases are unneccesary
            try: 
                loc = obj.carlaActor.get_location()
            except RuntimeError:
                print(f"Vehicle {obj.name} removed by CARLA likely due to deadlock")
                self.remove_bubble_object(obj, destroy=False)
                continue
            if (loc.x,loc.y,loc.z) == (0,0,0): # Carla object still in the process of spawning
                continue
            
            vehID = self.getMetsrPrivateVehId(obj)
            lane = self._nearest_lane(obj)
            roadID = self.map_scenic_to_metsr_lanes(lane)
            veh_data = all_veh_data[obj]
            # Update METSR road 
            if hasattr(obj, "previous_road"): 
                if roadID != obj.previous_road and roadID not in self.network_helper.intersection_road_links: # Entering new road within metsr network
                    self.metsr_client.enter_next_road(vehID, roadID=roadID, private_veh = True)
                    if obj.previous_road == obj.final_road:
                        obj.finished_route = True
            else:
                obj.finished_route = False
            # Record previous road location
            obj.previous_road = roadID
            bearing = get_metsr_rotation(obj.carlaActor.get_transform().rotation.yaw)
            # Check if objects are desynchronized
            if not math.isclose(loc.x, veh_data['x']) or not math.isclose(-loc.y, veh_data['y']):
                self.metsr_client.teleport_cosim_vehicle(vehID, loc.x, -loc.y, bearing=bearing, private_veh = True, transform_coords = True)



    def classify_bubble_roads(self, bubble_regions : list[Object] | None = None) -> tuple[list[Road], list[str], list[str]]:
        """
        Docstring for update_carla_roads
    
        :param bubble_regions: List of objects with a designated "bubble region" constituting the CoSim region
        :type obj: List[Object] or None

        Collects all roads which are intersecting the bubble region 
            (1) Default region is defined by the ego the region can be updated by passing objects with their corresponding regions
        """
        if bubble_regions is None:
            roads = self._get_bubble_roads() # Collect Road intersecting the bubble
        else:
            roads = []
            for region in bubble_regions:
                region_lanes = self._get_bubble_roads(bubble_region=region)
                roads.extend(region_lanes)

        self.frozen_metsr_roads = roads
        bubble_road_ids = []
        for road in roads:
            bubble_road_ids + self.map_scenic_to_metsr_road(road)
        self.frozen_roads = list(self.carla_control_roads.keys())
        # Collect roads into new and old for freeze/unfreezing 
        new_roads = [id for id in bubble_road_ids if id not in self.frozen_roads]
        old_roads = [id for id in self.frozen_roads if id not in bubble_road_ids]
        return roads, new_roads, old_roads
    
    
   
    def update_bubble_objects(self, bubble_roads: list[Road], intersections: list[Intersection]) -> None: 
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
            (iii) an equivalent Scenic trajectory can be constructed from the metsr proposed route
                
        """
        cosim_data = self.metsr_client.query_coSimVehicle()
        all_veh_data = self._collect_metsr_vehicle_data(self.objects[1:])

        for obj in self.objects[1:]: 
            veh_data = all_veh_data[obj]
          
            # Skip vehicles which have not entered the roadway or have completed their route 
            if ('road' not in veh_data) or obj.finished_route: 
                continue
            road = self._nearest_road(obj)
            intersection = self._get_intersection(obj, road)
            outside_bubble = (road not in bubble_roads and intersection not in intersections)
          
            # Spawn guard allows the client to process pending object creation """
            if obj.spawn_guard > 0: 
                obj.spawn_guard = max(obj.spawn_guard - self.sim_ticks_per, 0)
          
            # Remove vehicles which have left the cosimulation region and spawn vehicles which have entered
            if outside_bubble:
                if obj.carla_actor_flag:
                    self.remove_bubble_object(obj)
                else:
                    if obj in self.bubble_spawn_queue:
                        self.bubble_spawn_queue.remove(obj)
            else: 
                if not math.isclose(obj.position.x, veh_data['x']) or not math.isclose(obj.position.y, veh_data['y']): # enforce world state consistency
                    obj.position = Vector(veh_data["x"], veh_data["y"], 0)  

                if not obj.carla_actor_flag:    # Vehicle needs to be spawned 
                    not_enough_space = _utils.within_threshold_to(obj, self.carla_actors,verbose=False)
                    if not_enough_space:        # ensure there is sufficient room before spawning
                        if obj not in self.bubble_spawn_queue:
                            _utils.within_threshold_to(obj,self.carla_actors, verbose=False)
                            self.bubble_spawn_queue.add(obj)
                        continue
                    else:                        # spawn the vehicle
                        carla_trajectory, route_data = None, None
                        VehID = self.getMetsrPrivateVehId(obj) 
                        curr_lane = self._nearest_lane(obj)                   
                        for data_entry in cosim_data['DATA']:
                            if data_entry['ID'] == VehID:
                                route_data = data_entry['route']
                                trajectory = self.generate_scenic_trajectory(curr_lane, route_data)
                                if trajectory != None:
                                    obj.final_road = route_data[-1]
                                    carla_trajectory = self.scenic_trajectory_to_carla(trajectory) # Guarenteed that each vehicles trajectory starts at curent lane
                                    break                                                          # Once the trajectory is found continue  
                        if carla_trajectory == None:
                            if obj not in self.bubble_spawn_queue:
                                self.bubble_spawn_queue.add(obj)
                            continue # Do not spawn vehicle if no trajectory can be created
                        spawn_success = self.createObjectInCarla(obj,update_orientation=True, trajectory=carla_trajectory) 
                        if spawn_success == False:
                            self.bubble_spawn_queue.add(obj)
                            continue                   
                        elif obj in self.bubble_spawn_queue:
                            self.bubble_spawn_queue.remove(obj)
                    
                        self.carla_actors.append(obj)
                        self.metsr_actors.remove(obj)
            
    def scenic_trajectory_to_carla(self, trajectory: list[Lane]) -> list:
        """
        Docstring for scenic_trajectory_to_carla

        :param trajectory: Scenic trajectory starting at the vehicles location to their goal destinatino
        :type trajectory: list[Lane]

        Convert a list of scenic lanes to an equivalent sequence of CARLA waypoint locations
        
        Trajectories starting point must alwasy correspond to the vehicles current road
        """    
        way_points = []
        for i,lane in enumerate(trajectory):
            if i == 0:
                points = [lane.centerline.end]
            else:
                points = [lane.centerline.start, lane.centerline.end]
                for point in points:
                    scenic_pos = point
                    carla_rot = _utils.scenicToCarlaRotation(orientation=scenic_pos.orientation)
                    carla_loc = _utils.scenicToCarlaLocation(pos=scenic_pos)
                    way_point = carla.Transform(carla_loc, carla_rot)
                    way_points.append(way_point.location)
        return way_points


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
            obj.carlaActor = None # Set this to None to prevent reaccess a previously deleted vehicle? 
    
    def remove_bubble_object(self,obj, destroy=True) -> None:
        """
        Docstring for remove_bubble_object

        :param obj: object to be deleted
        :type obj: Car
        """
        if destroy:
            self.destroy_carla_obj(obj)
        obj.carla_actor_flag = False
        obj.trajectory = None
        self.carla_actors.remove(obj)
        self.metsr_actors.append(obj)   
    
    def destroy(self) -> None:
        """
        Docstring for destroy
        
        Destroy both simulators instances i.e (METSR, CARLA)
        """
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
    
    def map_scenic_to_metsr_lanes(self, lane: Lane) -> str:
        """Map Scenic lane to equivalent METSR road, guareneteed 1->1 mapping"""
        return self.network_helper.map_scenic_to_metsr_lanes(lane)
    
    def generate_scenic_trajectory(self, curr_lane : Lane , route: list[str]) -> list[Lane] | None:
        """Attempt to translate metsr route to equvialent sequence of Scenic lanes"""
        return self.network_helper.generate_scenic_trajectory(curr_lane, route)

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
            save_file = "metsr_state_at_{self.count}.bin"
        else:
            save_file = file_name
        
        self.metsr_client.save(save_file)


    """ light helpers for added functionality down the road"""

    def _check_traffic_light_consistency(self):
        """
        docstring for check_traffic_light_consistency
        """
        signals_ids = self.metsr_client.query_signal()['id_list']
        signal_data = self.metsr_client.query_signal(signals_ids)
        
        carla_world = self.carla_client.get_world()
        carla_traffic_lights = carla_world.get_actors().filter('traffic.traffic_light*')

        lights_by_opendrive_id = {light.get_opendrive_id(): light for light in carla_traffic_lights}
        for light_data in signal_data["DATA"]:
            light_id = light_data["groupID"]
            
            if light_id in self.xml_to_xodr_intersections:
                light_opendrive_ids = self.xml_to_xodr_intersections[light_id]
                light_config = self.get_light_config(light_data)
                if len(light_opendrive_ids) > 1:  
                    for open_drive_id in light_opendrive_ids:
                        if open_drive_id in lights_by_opendrive_id:
                            light = lights_by_opendrive_id[open_drive_id]
                            self._check_light_consistency(light, light_config)
                        else:
                            print(f'Unable to find associated light for opendrive_id : {open_drive_id}')

                else:
                    open_drive_id = light_opendrive_ids[0]
                    if open_drive_id in light_opendrive_ids:
                        light = lights_by_opendrive_id[open_drive_id]
                        self._check_light_consistency(light, light_config)
                    else:
                        print(f'Unable to find associated light for opendrive_id : {open_drive_id}')
            else:
                assert True, f"Failed to find corresponding intersection for METSR light: {light_id}"

           
        
    def _update_carla_light_state(self, light : carla.TrafficLight, light_config : dict[str: float | carla.libcarla.TrafficLightState]) -> None:
        """
        Docstring _update_carla_light_state

        :param light: Single Carla light instance
        :type light: Carla Light
        :param light_config: dictionary containing new ligh configuration
        :type light_config: dict
        
        Update the state of a light with a specified configuration
        """
        light.set_green_time(light_config['green_time'])
        light.set_yellow_time(light_config['yellow_time'])
        light.set_red_time(light_config['red_time'])
        light.set_state(light_config['state'])

 
    def _check_light_consistency(self, light : carla.TrafficLight, light_config : dict) -> None:
        """
        Checks that a light is configured according to a given config

        :param light: Single Carla light instance
        :type light: Carla Light
        :param light_config: dictionary containing expected light configuration
        :type light_config: dict
        """
        light_state_dict = _utils.get_carla_light_state(light)
        for key in light_config:
            if key in light_state_dict:
                if light_config[key] != light_state_dict[key]:
                    break
            else:
                assert True, f" Incomptible light states encountered due to key error for key {key}"
        
   
    def get_light_config(self, metsr_light_data : dict) -> dict[str: float]:
        """
        Docstring for get_light_config

        Generates the equivalent configuration for a Carla light given a metsr light instance

        :param metsr_light_data: Metsr query result for a single light
        :type metsr_light_data: dict
        """
        metsr_to_carla_states = {0 : carla.libcarla.TrafficLightState.Green, 1 : carla.libcarla.TrafficLightState.Yellow ,  2 : carla.libcarla.TrafficLightState.Red }

        light_config = {}
        # Assuming that State is consistent across a group TODO VERIFIY assumption
        light_config["green_time"] = metsr_light_data['phase_ticks'][0] * self.timestep
        light_config["yellow_time"] = metsr_light_data['phase_ticks'][1] * self.timestep
        light_config["red_time"] = metsr_light_data['phase_ticks'][2] * self.timestep
        light_config["state"] = metsr_to_carla_states[metsr_light_data['state']]

        self.metsr_client.update_signal(metsr_light_data['ID'], targetPhase=metsr_light_data['state'])
        return light_config


