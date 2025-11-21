from scenic.core.simulators import Simulation, Simulator
from scenic.simulators.metsr.simulator import METSRSimulator
from scenic.simulators.carla.simulator import CarlaSimulator
from scenic.core.vectors import Orientation, Vector
from scenic.syntax.veneer import verbosePrint
from scenic.simulators.metsr.client import METSRClient
import scenic.simulators.carla.utils.utils as utils


import pygame
import warnings
import os
import math
import numpy as np
from scenic.core.simulators import SimulationCreationError


import scenic.simulators.carla.utils.visuals as visuals
from scenic.simulators.carla.blueprints import oldBlueprintNames
from shapely.geometry import Point



try:
    import carla
except ImportError as e:
    raise ModuleNotFoundError('CARLA scenarios require the "carla" Python package') from e

# def initialize_Carla(map_name=None, map_path=None, address="127.0.0.1",port=2000,timeout=10):
#     verbosePrint(f"Connecting to CARLA on port {port}")
#     client = carla.Client(address,port)


class CosimSimulator(Simulator):
    def __init__(self, 
        metsr_map, 
        carla_map,
        map_path,
        bubble_size = 5, # random number should constitute the radius out from the ego object 
        carla_host="127.0.0.1", 
        carla_port=2000, 
        metsr_host="localhost", # Not sure what this actually means here
        metsr_port=4000, 
    #  timestep=1,# Not entirely sure what the distinction between timestep and sim_timestep is in metsr
        timestep=0.1,
        traffic_manager_port=None,
        timeout=10,
        verbose=False
    ):
        super().__init__()

        breakpoint()

        self.carla_map_name = carla_map
        self.metsr_map_name = metsr_map
        self.timestep = timestep
        self.sim_timestep = timestep
        self.map_path = map_path
        self.bubble_size = bubble_size

        # Setting up the Carla Simulator
        verbosePrint(f"Connection to CARLA on port {carla_port}")
        self.carla_client = carla
        carla.Client(carla_host,carla_port)
        self.carla_client.set_timeout(timeout)
        if carla_map is not None:
            try:
                self.world = self.carla_client.load_world(carla_map)
            except Exception as e:
                raise RuntimeError(f"CARLA could not load world '{carla_map}'") from e
        else:
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
        breakpoint()

        # Setting up Metsr simulator 

        self.metsr_client = METSRClient(host=metsr_host, port=metsr_port, verbose=verbose)

        verbosePrint("Clients have successfully been initialized")

    def createSimulation(self,scene,sim_timestep, **kwargs):
        assert sim_timestep is not None
        return CosimSimulation(
            scene=scene,
            carla_client=self.carla_client,
            metsr_client=self.metsr_client,
            sim_timestep=self.sim_timestep,
            tm=self.tm,
            bubble=self.bubble_size
        )
    def destroy(self):
        self.carla_sim.destroy()
        self.metsr_sim.destroy()
        super().destroy()

class CosimSimulation(Simulation):
    def __init__(self, scene, carla_client, metsr_client, sim_timestep, tm, bubble_size, **kwargs ):
    
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

        # Initializing METSR params
        self.next_pv_id = 0
        self.pv_id_map = {}
        self.frozen_vehicles = set()

        self._client_calls = []
        self.count = 0

        self.bubble = bubble_size

        super().__init__(scene, timestep=sim_timestep, **kwargs)



    def setup(self):
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

        # Set up camera manager and collision sensor for ego
        if self.render:
            camIndex = 0
            camPosIndex = 0
            egoActor = self.objects[0].carlaActor
            self.cameraManager = visuals.CameraManager(self.carla.world, egoActor, self.hud)
            self.cameraManager._transform_index = camPosIndex
            self.cameraManager.set_sensor(camIndex)
            self.cameraManager.set_transform(self.camTransform)

        self.carla_world.tick()  ## allowing manualgearshift to take effect    # TODO still need this?

        self.carla_actors = self.get_carla_actors() # initialize carla actors

        for obj in self.carla_actors:
            obj.carla_actor_flag = True
            if isinstance(obj.carlaActor, carla.Vehicle):
                obj.carlaActor.apply_control(
                    carla.VehicleControl(manual_gear_shift=False)
                )

        self.carla_world.tick()

        for obj in self.carla_objects:
            if obj.speed is not None and obj.speed != 0:
                raise RuntimeError(
                    f"object {obj} cannot have a nonzero initial speed "
                    "(this is not yet possible in CARLA)"
                )
        
        # Create a polygon buffer around the ego object
        self.objects[0].bubble = Point(self.objects[0].x, self.objects[1].y).buffer(self.bubble_size)


    def createObjectInMetsr(self, obj):
        assert obj.origin, "Metsr objects must have a defined origin"
        assert obj.destination, "Metsr objects must have a defined destination"

        call_kwargs = {
            "vehID": self.getPrivateVehId(obj),
            "origin": obj.origin,
            "destination": obj.destination,
        }

        self.metsr_client.generate_trip(**call_kwargs)


    def createObjectInCarla(self,obj):
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
                            f"CARLA blueprint {obj.blueprint} not found;"
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
        
        loc = utils.scenicToCarlaLocation(
            obj.postion,
            world=self.carla_world,
            blueprint=obj.blueprint,
            snapToGround=obj.snapToGround,
        )
        rot = utils.scenicToCarlaRotation(obj.orientation)
        transform = carla.Transform(loc,rot)

        if blueprint.has_attribute("color") and obj.color is not None:
            c = obj.color
            c_str = f"{int(c.r*255), {int(c.g*255)},{int(c.b*255)}}"
            blueprint.set_attribute("color", c_str)

        carlaActor = self.carla_world.try_spawn_actor(blueprint,transform)
        if carlaActor is None:
            raise SimulationCreationError(f"Unable to spawn object {obj}")
        obj.carlaActor = carlaActor

        carlaActor.set_simulate_physics(obj.physics)

        if isinstance(carlaActor, carla.Vehicle):
            extent = carlaActor.bounding_box.extent
            ex,ey,ez = extent.x, extent.y, extent.z

            obj.width = ey * 2 if ey > 0 else obj.width
            obj.lenth = ex * 2 if ex > 0 else obj.length
            obj.height = ez * 2 if ez > 0 else obj.height
            carlaActor.apply_control(carla.VehicleControl(manual_gear_shift=True, gear=1))

        elif isinstance(carlaActor, carla.Walker):
            carlaActor.apply_control(carla.WalkerControl())
            #spawn walker controller
            controller_bp = self.blueprintLib.find("controller.ai.walker")
            controller = self.carla_world.try_spawn_actor(
                controller_bp, carla.Transform(), carlaActor
            )
            if controller is None:
                raise SimulationCreationError(
                    f"Unable to spawn carla controller for object {obj}"
                )
            obj.carlaController = controller
        return carlaActor


    def getCarlaProperties(self, obj, properties):
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


    def getMetsrProperties(self, obj, properties):
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
            roll=roll
        )
        return values

    def getProperties(self, obj, properties):
        assert hasattr(obj, carla_actor_flag), f"Object is not assigned properly to a simulator instance"
        if obj.carla_actor_flag:
            return self.getCarlaProperties(obj)
        else:
            return self.getMetsrProperties(obj)


    def getMetsrPrivateVehId(self,obj):
        if obj not in self.pv_id_map:
            self.pv_id_map[obj] = self.next_pv_id
            self.next_pv_id += 1
        return self.pv_id_map[obj]


    def carla_step(self):
        """
        Applying step operations to be seperate for each simulator to allow 
        for more complex interations (differing frequencies) later on
        """
        self.carla_world.tick()

        if self.render:
            self.cameraManager.render(self.display)
            pygame.display.flip()

    
    def metsr_step(self):
        """
        Tick the metsr simulation a single timestep
        """
        self.count += 1
        if self.count % 100 == 0:
            print(".", end="", flush=True)
        self.metsr_client.tick()


    def step(self):
        """
        Step both simulators -> Update ego bubble 
            Update actor locations in either bubble
        """
        self.carla_client.tick()
        self.metsr_client.tick()
        # recreating a new bubble each time? Likely a better way to do this
        # Instead of using Shapely here -- use Scenic circle class 
        # Easier to use Scenic class (already have built in intersection stuff)
        self.objects[0].bubble = Point(self.objects[0].x, self.objects[1].y).buffer(self.bubble_size)
        # Circle objects region for bubble -- "ground truth"
        # Check lanes in 'bubble'
        # find the cars in the lanes  -- or if the car is the closest lane


        # lane =self.objects._lane 
        # if lane:
        #     min() -- 


        # query new actors that may have entered the bubble
        new_actors = self.get_carla_actors()
        # Check for new objs to initialize in Carla
        for new_actor in new_actors:
            if new_actor not in self.carla_actors:
                self.createObjectInCarla(new_actor)
                new_actor.carla_actor_flag = True
        # Remove objects that should no longer be in Carla
        for old_actor in self.carla_actors:
                if old_actor not in new_actors:
                    self.destroy_carla_obj(old_actor)
                    old_actor.carla_actor_flag = False
                    # release the road segment that the old actors was in originally
                    # This could potentially delete a segment that SHOULD still be contained
                    # This is OK because we will update with the relevant actors before ticking again
                    self.metsr_client.release_cosim_road(self.query_road(old_actor))


        self.carla_actors = new_actors
        self.query_road() #update the road segments which Metsr controls before ticking metsr



    def get_carla_actors(self):
        """
        Helper function to compute which objects belong in Carla
        """
        bubble = self.objects[0].bubble # Bubble around the ego
        actors = [self.objects[0]]
        for obj in self.objects[1:]: # check all objects in the sim
            mesh = self.objects[0].mesh # grab obj mesh
            boundary_points = mesh.vertices[mesh.vertices_on_boundary] # grab boundary points
            if np.any([bubble.touches(Point(p[:2])) for p in boundary_points]): #Check if any points are contained in buffer
                actors.append(obj) # add to the list of current actors
        return actors


    def query_road(self):
        """
            Query carla actors for relevent road segments which metsr will not control
        """
        for actor in self.carla_actors:
            road_id = self.get_mapping(actor.position)# placeholder idek how to do this
            self.metsr_client.set_cosim_road(road_id)


    def destroy_carla_obj(self,obj):
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

    
    def get_mapping(self,obj):
        """
        TODO: Map an objects position in xodr world to the metsr equivalent road ID
        """
        return 0

        