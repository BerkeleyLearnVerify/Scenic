
import math

import lgsvl
import dreamview
import numpy as np
from scipy import linalg

import scenic.simulators as simulators
import scenic.simulators.lgsvl.utils as utils
import scenic.syntax.veneer as veneer
from scenic.core.vectors import Vector

class LGSVLSimulator(simulators.Simulator):
    def __init__(self, lgsvl_scene, address='localhost', port=8181, alwaysReload=False):
        super().__init__()
        self.client = lgsvl.Simulator(address=address, port=port)
        if alwaysReload or self.client.current_scene != lgsvl_scene:
            self.client.load(scene=lgsvl_scene)

    def createSimulation(self, scene):
        return LGSVLSimulation(scene, self.client)


class LGSVLSimulation(simulators.Simulation):
    def __init__(self, scene, client):
        super().__init__(scene)
        self.client = client
        self.timeStep = scene.params.get('time_step', 1.0/30)
        self.objects = scene.objects
        self.usingApollo = False
        self.data = {}
        self.collisionOccurred = False

        # Reset simulator (deletes all existing objects)
        self.client.reset()

        # Create LGSVL objects corresponding to Scenic objects
        self.lgsvlObjects = {}
        for obj in self.objects:
            # Figure out what type of LGSVL object this is
            if not hasattr(obj, 'lgsvlObject'):
                continue    # not an LGSVL object
            if not hasattr(obj, 'lgsvlName'):
                raise RuntimeError(f'object {obj} does not have an lgsvlName property')
            if not hasattr(obj, 'lgsvlAgentType'):
                raise RuntimeError(f'object {obj} does not have an lgsvlAgentType property')
            name = obj.lgsvlName
            agentType = obj.lgsvlAgentType

            # Set up position and orientation
            state = lgsvl.AgentState()
            if obj.elevation is None:
                obj.elevation = self.groundElevationAt(obj.position)
            state.transform.position = utils.scenicToLGSVLPosition(obj.position, obj.elevation)
            state.transform.rotation = utils.scenicToLGSVLRotation(obj.heading)

            # Create LGSVL object
            lgsvlObj = self.client.add_agent(name, agentType, state)
            obj.lgsvlObject = lgsvlObj
            
            # Initialize Data
            self.data[obj] = {}
            # Initialize Apollo if needed
            if hasattr(obj, 'apolloVehicle'):
                self.initApolloFor(obj, lgsvlObj)

        # TODO reset object controllers???

    def groundElevationAt(self, pos):
        origin = utils.scenicToLGSVLPosition(pos, 100000)
        result = self.client.raycast(origin, lgsvl.Vector(0, -1, 0), 1)
        if result is None:
            print(f'WARNING: no ground at position {pos}')
            return 0
        return result.point.y

    def initApolloFor(self, obj, lgsvlObj):
        """Initialize Apollo for an ego vehicle.

        Uses LG's interface which injects packets into Dreamview."""
        if self.usingApollo:
            raise RuntimeError('can only use one Apollo vehicle')
        self.usingApollo = True

        def on_collision(agent1, agent2, contact):
            if agent1 is not None and agent1.name == lgsvlObj.name:
                self.data[obj]['collision'] = True
            if agent2 is not None and agent2.name == lgsvlObj.name:
                self.data[obj]['collision'] = True
            if self.data[obj]['collision']:
                self.collisionOccurred = True
                print('COLLISION !')

        # Initialize Data
        self.data[obj]['collision'] = False
        lgsvlObj.on_collision(on_collision)

        # connect bridge from LGSVL to Apollo
        lgsvlObj.connect_bridge(obj.bridgeHost, obj.bridgePort)

        # set up connection and map/vehicle configuration
        dv = dreamview.Connection(self.client, lgsvlObj)
        obj.dreamview = dv
        waitToStabilize = False
        hdMap = self.scene.params['apolloHDMap']
        if dv.getCurrentMap() != hdMap:
            print('previous map:',dv.getCurrentMap())
            print('current map:',hdMap)
            dv.setHDMap(hdMap)
            waitToStabilize = True
            print('map updated')
        if dv.getCurrentVehicle() != obj.apolloVehicle:
            dv.setVehicle(obj.apolloVehicle)
            waitToStabilize = True
            print('vehicle updated')
        
        print('Initializing Apollo...')

        # stop the car to cancel buffered speed from previous simulations
        cntrl = lgsvl.VehicleControl()
        cntrl.throttle = 0.0
        lgsvlObj.apply_control(cntrl, True)
        # start modules
        dv.disableModule('Control')
        #for module in obj.apolloModules:
        #    dv.disableModule(module)
        #self.client.run(5)
        ready = dv.getModuleStatus()
        for module in obj.apolloModules:
            if not ready[module]:
                dv.enableModule(module)
                print('Module', module, 'is not ready...')
                waitToStabilize = True
        while True:
            ready = dv.getModuleStatus()
            if all(ready[module] for module in obj.apolloModules):
                break

        # wait for Apollo to stabilize, if needed
        if waitToStabilize:
            print('waiting to stabilize')
            self.client.run(25)
        dv.enableModule('Control')
        self.client.run(15)
        print('Initialized Apollo.')

    def writePropertiesToLGSVL(self):
        for obj in self.objects:
            lgsvlObj = obj.lgsvlObject
            state = lgsvlObj.state
            # position, elevation
            state.transform.position = utils.scenicToLGSVLPosition(obj.position, y=obj.elevation)
            # heading
            state.transform.rotation = utils.scenicToLGSVLRotation(obj.heading)
            # update state
            lgsvlObj.state = state

    def readPropertiesFromLGSVL(self):
        for obj in self.objects:
            lgsvlObj = obj.lgsvlObject
            state = lgsvlObj.state
            # position, elevation
            obj.position = utils.lgsvlToScenicPosition(state.position)
            obj.elevation = utils.lgsvlToScenicElevation(state.position)
            # heading
            heading = utils.lgsvlToScenicRotation(state.rotation, tolerance2D=15)
            if heading is None:
                raise RuntimeError(f'{lgsvlObj} has non-planar orientation!')
            obj.heading = heading

    def currentState(self):
        return tuple(obj.position for obj in self.objects)

    def initialState(self):
        return self.currentState()

    def step(self, actions):
        # execute actions
        for agent, action in actions.items():
            if action is not None:
                action.applyTo(agent, agent.lgsvlObject, self)
        # run simulation for one time step
        self.client.run(time_limit=self.timeStep)
        # read back the results of the simulation
        self.readPropertiesFromLGSVL()
        return self.currentState()

class MoveAction(simulators.Action):
    def __init__(self, offset):
        self.offset = offset

    def applyTo(self, obj, lgsvlObject, sim):
        pos = obj.position.offsetRotated(obj.heading, self.offset)
        pos = utils.scenicToLGSVLPosition(pos, y=obj.elevation)
        state = lgsvlObject.state
        state.transform.position = pos
        lgsvlObject.state = state

class SetVelocityAction(simulators.Action):
    def __init__(self, velocity):
        self.velocity = utils.scenicToLGSVLPosition(velocity)

    def applyTo(self, obj, lgsvlObject, sim):
        state = lgsvlObject.state
        state.velocity = self.velocity
        lgsvlObject.state = state

class FollowWaypointsAction(simulators.Action):
    def __init__(self, waypoints):
        self.waypoints = tuple(waypoints)
        if not isinstance(self.waypoints[0], lgsvl.DriveWaypoint):
            pts = []
            for wp in self.waypoints:
                elev = veneer.simulation().groundElevationAt(wp)
                pos = utils.scenicToLGSVLPosition(wp, y=elev)
                rot = utils.scenicToLGSVLRotation(wp.heading)
                pt = lgsvl.DriveWaypoint(pos, wp.speed, rot)
                pts.append(pt)
            self.waypoints = tuple(pts)

        self.lastTime = -2

    def applyTo(self, obj, lgsvlObject, sim):
        #print(sim.currentTime, self.lastTime)
        if sim.currentTime is not self.lastTime + 1:
            agentType = obj.lgsvlAgentType
            if agentType in (lgsvl.AgentType.NPC, lgsvl.AgentType.PEDESTRIAN):
                lgsvlObject.follow(self.waypoints)
            else:
                raise RuntimeError('used FollowWaypointsAction with'
                                   f' unsupported agent {lgsvlObject}')
        self.lastTime = sim.currentTime

class CancelWaypointsAction(simulators.Action):
    def applyTo(self, obj, lgsvlObject, sim):
        lgsvlObject.walk_randomly(False)

class SetDestinationAction(simulators.Action):
    def __init__(self, dest):
        self.dest = dest
        self.timer = 0

    def applyTo(self, obj, lgsvlObject, sim):
        if self.timer == 0:
            print('Setting destination...')
            z = sim.groundElevationAt(self.dest)
            obj.dreamview.setDestination(self.dest.x, self.dest.y, z,
                                      coordType=dreamview.CoordType.Unity)

        # push vehicle for 1 second to start
        oneSec = int(1.0/sim.timeStep)
        if self.timer < oneSec:
            cntrl = lgsvl.VehicleControl()
            cntrl.throttle = 0.5
            lgsvlObject.apply_control(cntrl, True)
        elif self.timer == oneSec:
            print('Autopilot...')
            cntrl = lgsvl.VehicleControl()
            cntrl.throttle = 0.5
            lgsvlObject.apply_control(cntrl, False)
        self.timer = self.timer + 1


class TrackWaypoints(simulators.Action):
    def __init__(self, waypoints, cruising_speed = 10):
        self.waypoints = np.array(waypoints)
        self.curr_index = 1
        self.cruising_speed = cruising_speed

    def LQR(v_target, wheelbase, Q, R):
        A = np.matrix([[0, v_target*(5./18.)], [0, 0]])
        B = np.matrix([[0], [(v_target/wheelbase)*(5./18.)]])
        V = np.matrix(linalg.solve_continuous_are(A, B, Q, R))
        K = np.matrix(linalg.inv(R)*(B.T*V))
        return K

    def applyTo(self, obj, lgsvlObject, sim):
        state = lgsvlObject.state
        pos = state.transform.position
        rot = state.transform.rotation
        velocity = state.velocity
        th, x, y, v = rot.y/180.0*np.pi, pos.x, pos.z, (velocity.x**2 + velocity.z**2)**0.5
        #print('state:', th, x, y, v)
        PREDICTIVE_LENGTH = 3
        MIN_SPEED = 1
        WHEEL_BASE = 3
        v = max(MIN_SPEED, v)

        x = x + PREDICTIVE_LENGTH * np.cos(-th+np.pi/2)
        y = y + PREDICTIVE_LENGTH * np.sin(-th+np.pi/2)
        #print('car front:', x, y)
        dists = np.linalg.norm(self.waypoints - np.array([x, y]), axis=1)
        dist_pos = np.argpartition(dists,1)
        index = dist_pos[0]
        if index > self.curr_index and index < len(self.waypoints)-1:
            self.curr_index = index
        p1, p2, p3 = self.waypoints[self.curr_index-1], self.waypoints[self.curr_index], self.waypoints[self.curr_index+1]

        p1_a = np.linalg.norm(p1 - np.array([x, y]))
        p3_a = np.linalg.norm(p3 - np.array([x, y]))
        p1_p2= np.linalg.norm(p1 - p2)
        p3_p2= np.linalg.norm(p3 - p2)
        
        if p1_a - p1_p2 > p3_a - p3_p2:
            p1 = p2
            p2 = p3
        
        #print('points:',p1, p2)
        x1, y1, x2, y2 = p1[0], p1[1], p2[0], p2[1]
        th_n = -math.atan2(y2-y1,x2-x1)+np.pi/2
        d_th = (th - th_n + 3*np.pi) % (2*np.pi) - np.pi
        d_x = (x2-x1)*y - (y2-y1)*x + y2*x1 - y1*x2
        d_x /= np.linalg.norm(np.array([x1, y1]) - np.array([x2, y2]))
        #print('d_th, d_x:',d_th, d_x)


        K = TrackWaypoints.LQR(v, WHEEL_BASE, np.array([[1, 0], [0, 3]]), np.array([[10]]))
        u = -K * np.matrix([[-d_x], [d_th]])
        u = np.double(u)
        u_steering = min(max(u, -1), 1)
        
        K = 1
        u = -K*(v - self.cruising_speed)
        u_thrust = min(max(u, -1), 1)
        
        #print('u:', u_thrust, u_steering)
        
        cntrl = lgsvl.VehicleControl()
        cntrl.steering = u_steering
        if u_thrust > 0:
            cntrl.throttle = u_thrust
        elif u_thrust < 0.1:
            cntrl.braking = -u_thrust
        lgsvlObject.apply_control(cntrl, True)
        

