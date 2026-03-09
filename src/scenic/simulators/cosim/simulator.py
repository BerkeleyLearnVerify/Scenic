from scenic.core.simulators import Simulation, Simulator
from scenic.core.vectors import Orientation, Vector
from scenic.syntax.veneer import verbosePrint
from scenic.simulators.metsr.client import METSRClient
import scenic.simulators.carla.utils.utils as utils
from scenic.simulators.cosim.utils.utils import *
from scenic.core.regions import CircularRegion
from scenic.core.object_types import Object

from scenic.domains.driving.roads import Lane, Intersection

from scenic.simulators.carla.behaviors import *


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
        timeout=20,
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
    def __init__(self, scene, carla_client, metsr_client, sim_timestep, tm, render ,record, mappings, xml_to_xodr_intersections, bubble_size=100,  **kwargs ):
    
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
        self.queued_vehicles = {}
        self.bubble_spawn_queue = set({})
        self.frozen_scenic_lanes = []
        self.xml_to_xodr_intersections = xml_to_xodr_intersections

        super().__init__(scene, timestep=sim_timestep, **kwargs)



    def setup(self) -> None:
        """
        Docstring for setup

        Setup the simulation instance 
        """
        self.metsr_client.reset() # Updated version takes no arguements

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
        
        _, new_lanes, _ = self.update_carla_lanes() # TODO bad function name
        self.freeze_lanes(new_lanes)
        self.metsr_client.tick()
        
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
        self.metsr_client.generate_trip(**call_kwargs)


    def createObjectInCarla(self, obj: Object, update_orientation: bool = False, trajectory: list[carla.Transform] = None) -> None:
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
            snapToGround=obj.snapToGround,
        )
        lane = self._nearest_lane(obj)
        if update_orientation:
            lane = self._nearest_lane(obj)
            rot = utils.scenicToCarlaRotation(lane.orientation[obj.position])
        else: 
            rot = utils.scenicToCarlaRotation(obj.orientation)

        transform = carla.Transform(loc, rot)

        # Color, cannot be set for Pedestrians
        if blueprint.has_attribute("color") and obj.color is not None:
            c = obj.color
            c_str = f"{int(c.r*255)},{int(c.g*255)},{int(c.b*255)}"
            blueprint.set_attribute("color", c_str)

        # Create Carla actor
        # print(f"Spawning actor {obj} in location {loc} with original pos: {obj.position} in CARLA")
        try:
            carlaActor = self.carla_world.spawn_actor(blueprint, transform)
            obj.carla_actor_flag = True  
            obj.spawn_guard = 2 
        except Exception as e:
            print(f"Error: {e} occured \n displaying object positions")
            for car in self.objects:
                car_data = self.metsr_client.query_vehicle(self.getMetsrPrivateVehId(car), True, True)
                simulator = "carla" if car.carla_actor_flag else "metsr"
                if car.carla_actor_flag:
                    try:
                        loc = car.carlaActor.get_location()
                    except Exception as e:
                        loc = None

                    if loc is not None:
                        # print(f"{car} in {simulator}: [Metsr][Scenic][CARLA] coords X: {car_data['DATA'][0]['x'], car.position.x, loc.x}, y:{car_data['DATA'][0]['y'], car.position.y, -loc.y}")
                        print(f"Query Vehicle resuts for obj: :{car}, is: {car_data['DATA']}")
                    else:
                        # print(f"{car} in {simulator}: [Metsr][Scenic] coords X: {car_data['DATA'][0]['x'], car.position.x}, y:{car_data['DATA'][0]['y'], car.position.y}")
                        print(f"Query Vehicle resuts for obj: :{car}, is: {car_data['DATA']}")

            print(f"Checking distance function: {utils.within_threshold_to(obj,[obj for obj in self.objects if obj.carla_actor_flag], verbose=True)}")
            print(f"Issue occured at timestop: {self.count}")
            # raise e(f"Error : {e} occured")
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

        if obj == self.objects[0]: # ensures consistency for object queue spawning in METSR
            trajectory = None
            if hasattr(obj, "trajectory"):
                if obj.trajectory is not None:
                    trajectory = self.scenic_trajectory_to_carla(trajectory)
            lane = self._nearest_lane(obj)
            origin_str = self.map_scenic_to_metsr(lane)
            if origin_str[0] == ":": # special handling for ":" suffix on specific roads
                origin = int(origin_str[1:])
            else:
                origin = int(origin_str)
            self.createObjectInCarla(obj, False, trajectory)
            self.createObjectInMetsr(obj,origin=origin) # Set the METSR vehicle origin to match ego spawn
            obj.final_road = None
            self.synchronize_clients(obj) # First time ego is created synchronize clients
            if self.count == 0:
                self.metsr_client.tick()

        elif obj.carla_actor_flag:
            self.createObjectInCarla(obj)
            self.synchronize_clients(obj)
            obj.finished_route = False
        else:
            self.createObjectInMetsr(obj)
            obj.finished_route = False
            if self.count == 0:
                self.metsr_client.tick()



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

    def getProperties(self, obj : Object, properties : dict)-> dict[str, float | Vector | int]:
        """
        Docstring for getProperties

        :param obj: Cosimulation car object
        :type obj: Scenic Object
        
        return objects properties for any CoSim object
        """
        assert hasattr(obj, "carla_actor_flag"), f"Object is not assigned properly to a simulator instance"
        """ TODO: Sometimes update objects fails on the first step -- Not clear why this occurs -- adding a check before accessing obj.carlaActor """
        if obj.carla_actor_flag: 
            properties = self.getCarlaProperties(obj,properties)
        else:
            properties =  self.getMetsrProperties(obj,properties)
        return properties

    def getMetsrPrivateVehId(self, obj: Object) -> int:
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
        intersections = self.get_carla_intersections()
        # Update state of simulator with new bubble region
        self.update_bubble_objects(lanes, new_lanes, intersections)

        #Carla step
        self.carla_world.tick() 
        self.synchronize_clients()

        # It takes 2 CARLA ticks to fully instantiate and update properties once spawned in CARLA 
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

        # Generate bubble region based on ego objects TODO update logic for multiple high-fidelity zones
        self.objects[0].bubble = CircularRegion(center=[self.objects[0].x,
                                                        self.objects[0].y],
                                                        radius=self.bubble_size)
        if self.objects[0].interrupt:
            _utils.disable_carla_autopilot(self.objects[0])

        # if self.count % 10 == 0:
        #     self._check_traffic_light_consistency()
        

    def get_carla_lanes(self) -> list[Lane]:
        """
        Collect the current set of lanes intersecting the CoSimulation bubble
        """
        # Set ego, lane, bubble
        ego = self.objects[0]
        lane = self._nearest_lane(ego)
        bubble_region = ego.bubble 

        # Collect lanes which intersect bubble
        carla_lanes = []
        for lane in self.workspace.network.lanes:
            if lane.intersects(bubble_region):
                carla_lanes.append(lane) 

        return carla_lanes
    
    def get_carla_intersections(self) -> list[Intersection]:
        """
        Collect any intersections that are either
            (1) Intersecting the CoSim bubble
            (2) Are connected to the bubble via AT LEAST 2 roads
   
        :return: The set of all intersections meeting the above criteria
        :rtype: list[Intersection]
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
                continue
            intersection_roads = intersection.roads
            count = 0
            for road in intersection_roads:
                if road in self.frozen_scenic_lanes:
                    count += 1
                if count > 1:
                    carla_intersections.append(intersection)
                    break
            
        return carla_intersections
    
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
                    print(f"Carla: {light_state_dict}, Metsr: {light_config}")
                    break
                # assert light_config[key] == light_state_dict[key], f" Contradicting light states encountered: current light : {light_state_dict} proposed state {light_config}"
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
            carla_actors = [obj for obj in self.objects if obj.carla_actor_flag]
            carla_actors.append(self.objects[0])
        
        for obj in carla_actors:
            loc = obj.carlaActor.get_location()
            vehID = self.getMetsrPrivateVehId(obj)
            lane = self._nearest_lane(obj)
            roadID = self.map_scenic_to_metsr(lane)
            veh_data = self.metsr_client.query_vehicle(self.getMetsrPrivateVehId(obj), True, True) 
            # Update METSR road 
            if hasattr(obj, "previous_road"): 
                if roadID != obj.previous_road:
                    print(f"obj: {obj.name} leaving road: {obj.previous_road} moving to : {roadID}")
                    # Find the corresponding METSR keys
                    carla_lane_ids = set([self.map_scenic_to_metsr(lane) for lane in self.frozen_scenic_lanes])
                    if roadID not in carla_lane_ids:
                        print(f"Vehicle is leaving the bubble region")
                    else:
                        print(f"Vehicle remains in the bubble region")
                    self.metsr_client.enter_next_road(vehID, roadID=roadID, private_veh = True)
                    if obj.previous_road == obj.final_road:
                        obj.finished_route = True
            else:
                obj.finished_route = False
            # Record previous road location
            obj.previous_road = roadID
            bearing = get_metsr_rotation(obj.carlaActor.get_transform().rotation.yaw)
            # Check if objects are desynchronized
            if not math.isclose(loc.x, veh_data["DATA"][0]['x']) or not math.isclose(-loc.y, veh_data["DATA"][0]['y']):
                self.metsr_client.teleport_cosim_vehicle(vehID, loc.x, -loc.y, bearing=bearing,private_veh = True, transform_coords = True)



    def update_carla_lanes(self, bubble_regions : list[Object] | None = None) -> tuple[list[Lane], list[str], list[str]]: #TODO break this out into multiple functions/helpers
        """
        Docstring for update_carla_lanes
    
        :param bubble_regions: List of objects with a designated "bubble region" constituting the CoSim region
        :type obj: List[Object] or None

        Collects all lanes which are intersecting the bubble region 
        (1) Default region is defined by the ego the region can be updated by passing objects with their corresponding regions
        """
        if bubble_regions is None:
            # Collect lanes intersecting the bubble
            lanes = self.get_carla_lanes() 
            self.frozen_scenic_lanes = lanes
            # Find the corresponding METSR keys
            carla_lane_ids = [self.map_scenic_to_metsr(lane) for lane in lanes] 
            # scenic_lane_ids = [lane.road for lane in lanes]
            carla_lane_ids = set(carla_lane_ids) 
            # Lanes which are already set
            curr_frozen_ids = list(self.carla_control_roads.keys())
        
            # Collect new and old lanes 
            new_lanes = [id for id in carla_lane_ids if id not in curr_frozen_ids]
            old_lanes = [id for id in list(self.carla_control_roads.keys()) if id not in carla_lane_ids]
            
            # Update object existance based on bubble changes
        return lanes, new_lanes, old_lanes
    
    
   
    def update_bubble_objects(self, carla_lanes: list[Lane], new_lanes: list[str], intersections: list[Intersection]) -> None: 
        """
        Docstring for update_bubble_objects
        
        :param carla_lanes: a list of Scenic lanes which constitute the cosimulation region 
        :type carla_lanes: list[Lane]
        :param intersections: a list of Scenic intersections which are contained or touching the cosimulated region
                              (A lane must either intersect or have to connecting roads in the cosimulation region)
        :type intersections: list[intersection]

        (1) Remove all objects that form CARLA which have either
            (i) finished their associated route
            (ii) left the region
        (2) Spawn new objects in the Cosimulation region if
            (i)   Their is enough room in the obj's current location to spawn
            (ii)  The vehicle is not currently waiting to spawn in a metsr queue
            (iii) an equivalent Scenic trajectory can be constructed from the metsr proposed route
                
        """
        carla_actors = [obj for obj in self.objects if obj.carla_actor_flag] # TODO might need to consider a more efficient data structure here
        cosim_data = self.metsr_client.query_coSimVehicle()
        for obj in self.objects[1:]: 
            veh_data = self.metsr_client.query_vehicle(self.getMetsrPrivateVehId(obj), True, True) #
            lane = self._nearest_lane(obj)
            intersection = obj._intersection

            if obj.carla_actor_flag:
                if obj.finished_route:
                    carla_actors.remove(obj)
                    print(f'removing vehicle: {obj.name} after completing its route')
                    print(f"Current road_lane (Scenic) is {f'{lane.road.id}_{lane.id}'}")
                    print(f"{veh_data['DATA'][0]}")
                    self.remove_bubble_object(obj)
                
                elif (lane not in carla_lanes) and (intersection not in intersections):
                    if obj.spawn_guard == 0:
                        self.remove_bubble_object(obj)
                        carla_actors.remove(obj)
           
            # Add objects to the bubble if they are entering through a new lane and their is not enough space to spawn
            elif not obj.carla_actor_flag and (lane in carla_lanes or intersection in intersections):
                if 'dist' not in veh_data["DATA"][0] or obj.finished_route: # Check that vehicle is not waiting in queue
                    continue
                
                not_enough_space = _utils.within_threshold_to(obj, carla_actors)

                if not_enough_space:
                    # print(f"Not enough space to spawn {obj.name} at {self.count}:")
                    # for obj in self.objects:
                    #     car_data = self.metsr_client.query_vehicle(self.getMetsrPrivateVehId(obj), True, True)
                    #     simulator = "carla" if obj.carla_actor_flag else "metsr"
                    #     if obj.carla_actor_flag:
                    #         print(f"{obj} in {simulator}: [Metsr][Scenic] coords X: {car_data['DATA'][0]['x'], obj.position.x}, y:{car_data['DATA'][0]['y'], obj.position.y}")
                    continue

                # If their is enough room check that the vehicle is not currently queued and spawn  
                else:
                    carla_trajectory, route_data = None, None
                    VehID = self.getMetsrPrivateVehId(obj)
                    for data_entry in cosim_data['DATA']:
                        if data_entry['ID'] == VehID:
                            route_data = data_entry['route']
                            trajectory = self.generate_scenic_trajectory(lane, route_data, obj._intersection)
                            if trajectory != None:
                                obj.final_road = route_data[-1]
                                carla_trajectory = self.scenic_trajectory_to_carla(trajectory)
                                break # Once the trajectory is found continue

                    if carla_trajectory == None:
                        if obj not in self.bubble_spawn_queue:
                            print(f"No valid trajectory found for vehicle data: {[data_entry['route'] for data_entry in cosim_data['DATA'] if data_entry['ID'] == VehID]} ")
                            print(f"Skipping spawn for obj: {obj.name} at {lane.road.id}_{lane.id} with intersection {obj._intersection}: due to failed trajectory generation")
                            print(f"Obj location was: :{obj.x, obj.y} default lane was: {obj._lane.id if obj.lane != None else None} selected lane was: {lane}")
                            self.bubble_spawn_queue.add(obj)
                        continue # Do not spawn vehicle if no trajectory can be created

                    if obj._intersection in intersections:
                        print(f"Spawning object {obj} in intersection")
                    
                    print(f"Spawning obj: {obj.name} in CARLA at : {obj.x, obj.y}")
                    self.createObjectInCarla(obj,update_orientation=True, trajectory=carla_trajectory)
                    carla_actors.append(obj)

            
    def scenic_trajectory_to_carla(self, trajectory: list[Lane]) -> list:
        """
        Docstring for scenic_trajectory_to_carla

        :param trajectory: Scenic trajectory starting at the vehicles location to their goal destinatino
        :type trajectory: list[Lane]

        Convert a list of scenic lanes to an equivalent sequence of CARLA waypoint locations
        """    
        way_points = []
        # world = self.carla_client.get_world()
        for lane in trajectory:
            points = [lane.centerline.start, lane.centerline.end]
            for point in points:
                scenic_pos = point
                carla_rot = _utils.scenicToCarlaRotation(orientation=scenic_pos.orientation)
                carla_loc = _utils.scenicToCarlaLocation(pos=scenic_pos)
                way_point = carla.Transform(carla_loc, carla_rot)
                way_points.append(way_point.location)
        return way_points


    def freeze_lanes(self, keys: list[str]) -> None:
        """
        Docstring for freeze_lanes
        
        :param keys: RoadIDs for METSR indexed roads
        :type keys: list[str]
        
        Query Metsr to freeze simulation and control of given lanes

        """
        keys = set(keys)
        for key in keys:
            assert key not in self.carla_control_roads, "Attempted to freeze already frozen lane"
            self.carla_control_roads[key] = True    # Keep track of frozen lanes
            print(f"Freezing METSR key: {key}")
            self.metsr_client.set_cosim_road(key)


    def release_lanes(self,keys: list[str]) -> None:
        """
        Docstring for release_lanes
        
        :param keys: RoadIDs for METSR indexed roads
        :type keys: list[str]
        
        Query Metsr to begin re-simulating and control given lanes
        """
        keys = set(keys)
        for key in keys:
            assert key in self.carla_control_roads, "Attempted to release non frozen lane"
            del self.carla_control_roads[key] # Remove frozen lane from record
            print(f"Releasing METSR key: {key}")
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
    
    def remove_bubble_object(self,obj) -> None:
        """
        Docstring for remove_bubble_object

        :param obj: object to be deleted
        :type obj: Car
        """
        obj.carla_actor_flag = False
        self.destroy_carla_obj(obj)
        obj.trajectory = None

    
    def map_scenic_to_metsr(self,lane: Lane) -> str:
        """
        Docstring for map_scenic_to_metsr

        :param lane: Lane object to be mapped
        :type lane: Lane
        :return: Takes a Lane object and computes the corresponding METSR road which holds that lane
        :rtype: str
        """
        metsr_key=None
        # Parent road key with associated lane id
        query_key = f'{lane.road.id}_{lane.id}' 
        
        # Check if element is present in map between formats
        if query_key in self.scenic_to_metsr_map: 
            metsr_key = self.scenic_to_metsr_map[query_key]
        
        metsr_key = metsr_key.split("_")[0]

        # There must be a valid mapping 
        assert metsr_key is not None, f"Error identifying associated ID for {query_key}"    
        return metsr_key
    
    def generate_scenic_trajectory(self, curr_lane : Lane , route: list[str], intersection: Intersection = None) -> list[Lane]:
        """
        Docstring for generate_scenic_trajectory
        
        :param curr_lane: Current lane which the target object is placed on 
        :type curr_lane: Lane
        :param route: Metsr route data for a single car
        :type route: list[str]
        :param intersection: Current intersection object is on if any
        :type intersection: Intersection | None
        :return: Equivalent trajetory with Scenic Lanes
        :rtype: list[Lane]


        TODO: This is inefficient I think it will be beforehand generate some of these pairings rather than every
              time a new car is spawned. Also -- I am unsure how to enforce trajectory feasibility at the lane level? 

        """ 
        target_start = []
        if intersection is not None:
            intersection_roads = intersection.roads
            for road in intersection_roads:
                target_start.append(str(road.id))

        # For each road find the corresponding {road}_{lane} pair
        map_data = self.scenic_to_metsr_map.items()
        valid_lanes = {}
        for road in route:
            for scenic_key, metsr_key in map_data:
                    key_road = metsr_key.split("_")[0]
                    if key_road == road:
                        if road not in valid_lanes:
                            valid_lanes[road] = []
                        valid_lanes[road].append(scenic_key)

        starting_points = [lane.split("_")[0] for lane in valid_lanes[route[0]]]
        target_start = [str(curr_lane.road.id)]
        is_valid_trajectory = False
        
        for target in target_start:
            if target in starting_points:
                is_valid_trajectory = True
                break
        if not is_valid_trajectory:
            return None

        trajectory = []        
        lanes = [*self.workspace.network.lanes]  
        for i,road in enumerate(route):
            target_lanes = valid_lanes[road]
            assert len(target_lanes) > 1,  f"Failed to find target lanes for road: {road}"
            for road_lane in target_lanes:
                if len(trajectory) == i+1:
                    break
                for lane in lanes:
                    scenic_road = f'{lane.road.id}'
                    query_road = road_lane.split("_")[0]
                    query_lane = road_lane.split("_")[1]
                    opposite_traffic_flag = bool(query_lane[0] == "-")
                    if query_road == scenic_road:
                        if opposite_traffic_flag and str(lane.id)[0] == "-":
                            trajectory.append(lane)
                            break
                        elif not opposite_traffic_flag and not str(lane.id)[0] == "-":
                            trajectory.append(lane)
                            break
      
        if len(trajectory) < 1:
            return None

        # print(f"Found trajectory: {[f'{lane.road.id}_{lane.id}' for lane in trajectory]} for route {route}")
            
        return trajectory
        
    
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
        super().destroy()


    def _nearest_lane(self,obj, allow_offlane=True) -> Lane | None : # TODO :: Update lane logic to consider intersections
        """
        Docstring for _nearest_lane
        
        Return the nearest lane to the object

        TODO: Allow a user specified mode where cars are NOT allowed to leave the road? 
        TODO: Ensure all objects are cars
        """
        lane = obj._lane
        if lane:
            nearest_lane = lane
        else:
            if not allow_offlane:
                assert True, f"Object: {obj.name} is has left the roadway"
            lanes = [*self.workspace.network.lanes]            
            distances = [(lane.distanceTo(obj.position),  lane) for lane in lanes]
            nearest_lane = min(distances, key=lambda t: t[0])[1] # min distance over all lanes
        
        return nearest_lane

    
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
        metsr_obj = [obj for obj in self.objects if not obj.carla_actor_flag]
        metsr_obj.append(self.objects[0])
        obj_veh_ids = [self.getMetsrPrivateVehId(obj) for obj in self.objects]
        raw_veh_data = self.metsr_client.query_vehicle(obj_veh_ids, True, True)
        self.obj_data_cache = {obj: raw_veh_data['DATA'][i] for i, obj in enumerate(self.objects)}

        # #DEBUGGING FOR METSR 
        # for obj in self.obj_data_cache:
        #     if 'dist' not in self.obj_data_cache[obj]:
        #         self.queued_vehicles[obj] = True
        #     elif 'dist' in self.obj_data_cache[obj] and obj in self.queued_vehicles:
        #         print(f"obj {obj} leaving the spawn queue")
        #         del self.queued_vehicles[obj] 

        super().updateObjects()
        self.obj_data_cache = None 

    def check_world_state_consistency(self):
        """
        Docstring for check_world_state_consistency
        
        self: CoSimulation Object

        Compares the state of each vehicle in each simulator
        Displays any vehicles which are outside of the tolerance threshold
        """
        for obj in self.objects:
            if obj.carla_actor_flag:
                car_data = self.metsr_client.query_vehicle(self.getMetsrPrivateVehId(obj), True, True)
                metsr_x, metsr_y = car_data['DATA'][0]['x'], car_data['DATA'][0]['y']
                loc = obj.carlaActor.get_location()
                carla_x, carla_y = loc.x, loc.y
                in_metsr_queue = not bool("dist" in car_data['DATA'][0])

                if not np.isclose(metsr_x, carla_x):
                    print(f"Checking Object synchronization before Stepping")
                    print(f"Obj: {obj} in metsr queue? {in_metsr_queue}")
                    print("=" * 25)
                    lane = obj._lane
                    if lane:
                        print(f"OBJ lane: {self.map_scenic_to_metsr(obj._lane)}")
                    print(f"OBJ X: {obj}| METSR {metsr_x}: CARLA {carla_x}")
                if not np.isclose(metsr_y, -carla_y):
                    print(f"OBJ Y: {obj}| METSR {metsr_y}: CARLA {-carla_y}")
                    print("=" * 25)
            
        metsr_data = self.metsr_client.query_coSimVehicle()
        print(f"Displaying METSR query data: {metsr_data}")


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



# Leftovers I am preserving in case they become useful
"""
        # metsr_light_group = {}

        # for opendrive_id in list(lights_by_opendrive_id.keys()):
        #     associated_lights = []
        #     for light_data in signal_data["DATA"]:
        #         key = light_data["groupID"]
        #         targets= self.xml_to_xodr_intersections[key]
        #         if len(targets) > 1:
        #             for target in targets:
        #                 if target == opendrive_id:
        #                     associated_lights.append(light_data["ID"])
        #         else:
        #             target = targets[0]
        #             if target == opendrive_id:
        #                 associated_lights.append(light_data["ID"])
          
        #     metsr_light_group[opendrive_id] = associated_lights

        #     print(f"Displaying configs for : {opendrive_id}")
        #     for id in metsr_light_group[opendrive_id]:
        #         data = self.metsr_client.query_signal(id)["DATA"][0]
        #         light_config = self.get_light_config(data)
        #         print(light_config)
        #     print(f"==========================================")
                
        # # print(f"finishing consistency check at : {self.count}")

"""