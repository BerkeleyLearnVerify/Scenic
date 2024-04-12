from verifai.simulators.webots.webots_task import webots_task
from verifai.simulators.webots.client_webots import ClientWebots
try:
    from controller import Supervisor
except ModuleNotFoundError:
    import sys
    sys.exit('This functionality requires webots to be installed')

from dotmap import DotMap
import numpy as np

# Defining the task as a webots task
class cnn_cones_lanechange(webots_task):
    def __init__(self, N_SIM_STEPS, supervisor):
        super().__init__(N_SIM_STEPS, supervisor)
        self.traj_x = np.zeros(self.N_SIM_STEPS)
        self.traj_z = np.zeros(self.N_SIM_STEPS)

    def use_sample(self, sample):
        print('Sample recieved')
        print(sample)
        def get_rot_vector(direction):
            '''Convert discrete direction in rotation vector'''
            return {
                '0': [0,1,0,0],
                '1': [1,0,0,1.57],
                '2': [0,0,1,1.57],
                '3': [-1,0,0,1.57],
                '4': [0,0,-1,1.57],
            }[str(direction)]

        ego_car = self.supervisor.getFromDef('EGOCAR')

        reaction_time = sample.control_params.reaction_time[0]
        cruising_speed = sample.control_params.cruising_speed[0]
        x_init = sample.control_params.x_init[0]
        broken_car_color = sample.env_params.broken_car_color
        broken_car_rotation = sample.env_params.broken_car_rotation[0]

        controller_args = str(reaction_time) + " " + str(cruising_speed)
        ego_pos = ego_car.getField('translation').getSFVec3f()

        ego_car.getField('translation').setSFVec3f([ego_pos[0]+ x_init, ego_pos[1], ego_pos[2]])
        ego_car.getField('controllerArgs').setSFString(controller_args)
        ego_car.restartController()

        broken_car = self.supervisor.getFromDef('BROKENCAR')
        broken_car.getField('color').setSFColor(list(broken_car_color))
        broken_car.getField('rotation').setSFRotation([0, 1, 0, broken_car_rotation])


        traffic_cones_pos_noise = sample.cones_config.traffic_cones_pos_noise
        traffic_cones_down = [sample.cones_config.traffic_cones_down_0,
                              sample.cones_config.traffic_cones_down_1,
                              sample.cones_config.traffic_cones_down_2]
        self.traffic_cones_pos = []
        for i in range(3):
            traffic_cone = self.supervisor.getFromDef('TRAFFICCONE' + str(i))
            traffic_cone_pos = traffic_cone.getField('translation').getSFVec3f()
            traffic_cone.getField('translation').setSFVec3f([traffic_cone_pos[0]+
                                                             traffic_cones_pos_noise[i],
                                                             traffic_cone_pos[1],
                                                             traffic_cone_pos[2]])
            traffic_cone.getField('rotation').setSFRotation(get_rot_vector(traffic_cones_down[i]))
            self.traffic_cones_pos.append([traffic_cone_pos[0]+ traffic_cones_pos_noise[i],
                                           traffic_cone_pos[2]])
        self.dt = 0.032


        return ego_car

    def run_task(self, sample):
        ego_car = self.use_sample(sample)
        for i in range(self.N_SIM_STEPS):

            x_ego, _, z_ego = ego_car.getField('translation').getSFVec3f()
            self.traj_x[i] = x_ego
            self.traj_z[i] = z_ego

            self.supervisor.step(1)

        # Saving necessary trajectories for monitor
        traj = {}

        for i in range(3):
            cone_pos = self.traffic_cones_pos[i]
            dist_from_cone = np.sqrt((self.traj_x - cone_pos[0]) ** 2 +
                                     (self.traj_z + 1 - cone_pos[1]) ** 2)

            traj['collisioncone' + str(i)] = [(j*self.dt, v-1.8) for j, v in enumerate(dist_from_cone)]

        traj['overshoot'] = [(j*self.dt, (-3.0-v)) for j,v in enumerate(self.traj_x)]
        traj['stabilize'] = [(j*self.dt, (0.1-abs(2.5+v))) for j, v in enumerate(self.traj_x)]

        sim_results = traj

        return sim_results



PORT = 8888
BUFSIZE = 4096
N_SIM_STEPS = 500
supervisor = Supervisor()
simulation_data = DotMap()
simulation_data.port = PORT
simulation_data.bufsize = BUFSIZE
simulation_data.task = cnn_cones_lanechange(N_SIM_STEPS=N_SIM_STEPS, supervisor=supervisor)
client_task = ClientWebots(simulation_data)
if not client_task.run_client():
    print("End of simulations")
    supervisor.simulationQuit(True)
