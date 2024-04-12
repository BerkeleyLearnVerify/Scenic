from verifai.simulators.car_simulator.examples.control_utils.LQR_computation import *
from verifai.simulators.car_simulator.simulator import *
from verifai.simulators.car_simulator.lane import *
from verifai.simulators.car_simulator.car_object import *
from verifai.simulators.car_simulator.client_car_sim import *
import numpy as np
from dotmap import DotMap

def controller(x_trajectory, u_trajectory, control_params):
    x, y, v, theta = x_trajectory[-1]
    wheelbase = control_params.wheelbase
    a_star = control_params.a_star
    v_star = control_params.v_star
    control_freq = control_params.control_freq
    dt = control_params.dt
    Q = control_params.Q
    R = control_params.R
    A, B = extract_AB(speed=v, dt=dt, wheelbase=wheelbase)
    if len(u_trajectory)%control_freq == 0:
        a = a_star if np.linalg.norm(v - v_star) > 0.1 else 0.0
        c = np.array([0.0, a * control_freq * dt, 0.0])
        K, k = discrete_LQR(A, B, Q, R, c)
        u = K.dot(np.array([[x],
                            [v - v_star],
                            [-theta+np.pi/2]])) + k
        u = min(float(u), np.pi / 4.)
        u = max(float(u), -np.pi / 4.)
        control = np.array([u, a])
    else:
        control = u_trajectory[-1]
    return control


def lanekeeping(sample, control_params, width=0.13):
    x_init = sample.init_conditions.x_init[0]
    theta_init = -sample.init_conditions.theta_init[0] + np.pi/2
    v_init = 0.0
    y_init = 0.0
    x0 = np.array([x_init, y_init, v_init, theta_init])
    a_star = control_params.a_star

    u_domain = {'omega':[-np.pi/4, np.pi/4],
                'acc':[-a_star, a_star]}
    compute_control = lambda x, u: controller(x_trajectory=x, u_trajectory=u,
                                              control_params=control_params)

    car = bicycle_model(x0=x0, u_domain=u_domain, compute_control=compute_control,
                        wheelbase = control_params.wheelbase, dt = control_params.dt,
                        color='red')

    lanes = []
    lanes.append(straight_lane([0., -1.], [0., 1.], width))
    lanes.append(lanes[0].shifted(1))
    lanes.append(lanes[0].shifted(-1))

    world = simulator_world(lanes=lanes, cars=[car])

    sim = simulator(dt=control_params.dt, iters = 100, sprite_scale=control_params.sprite_scale,
                    window_size = control_params.window_size, magnify =0.25)
    sim.init_simulator(world=world, task_name='Lane Keeping')
    sim.run()
    sim.exit_simulation()
    traj_x, traj_y, _, _ = np.array(car.trajectory).T
    data = {}
    traj = {}
    traj['xdeviation'] = [(j * control_params.dt, 0.5 - np.abs(v)) for j, v in enumerate(traj_x)]
    return traj

def lanekeeping_simulator(sample):
    print(sample)
    control_params = DotMap()
    control_params.wheelbase =2.995
    control_params.a_star = 3.968
    control_params.v_star = sample.init_conditions.cruising_speed[0]*5./18.
    control_params.dt = 0.032
    control_params.control_freq = 2
    control_params.R = 50*np.identity(1)
    control_params.Q= np.diag([100.0, 0.0, 5.0])
    control_params.sprite_scale = 1/800
    control_params.window_size = 800
    width = 1.0
    return lanekeeping(sample, control_params, width)


PORT = 8888
BUFSIZE = 4096
N_SIM_STEPS = 100
simulation_data = DotMap()
simulation_data.port = PORT
simulation_data.bufsize = BUFSIZE
simulation_data.simulation = lanekeeping_simulator

client_task = ClientCar(simulation_data)
while True:
    if not client_task.run_client():
        print("End of all simulations")
        break









