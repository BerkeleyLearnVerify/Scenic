# This file has examples of simple control

# Example 1 : Simple rendering of a scene with a car
from verifai.simulators.car_simulator.simulator import *
from verifai.simulators.car_simulator.lane import *
from verifai.simulators.car_simulator.car_object import *
import numpy as np

def stalled_car(width=0.13):
    lanes = []
    lanes.append(straight_lane([0., -1.], [0., 1.], width))
    lanes.append(lanes[0].shifted(1))
    lanes.append(lanes[0].shifted(-1))

    # Defining a standing car
    cars = []
    car = bicycle_model(x0= np.array([0.0, 0.0, 0.0, np.pi/2]),
                        u_domain= {'omega':[-np.pi/4, np.pi/4],
                                   'acc':[-4.0, 4.0]},
                        compute_control= lambda x, u: np.array([0.0, 0.0]),
                        color='red')
    cars.append(car)

    world = simulator_world(lanes=lanes, cars=cars)

    sim = simulator()
    sim.init_simulator(world=world, task_name='Stalled Car')
    sim.run()
    sim.exit_simulation()

def constant_speed_car(width=0.13, speed=1.0):
    lanes = []
    lanes.append(straight_lane([0., -1.], [0., 1.], width))
    lanes.append(lanes[0].shifted(1))
    lanes.append(lanes[0].shifted(-1))

    # Defining a standing car
    cars = []
    car = bicycle_model(x0= np.array([0.0, 0.0, speed, np.pi/2]),
                        u_domain= {'omega':[-np.pi/4, np.pi/4],
                                   'acc':[-4.0, 4.0]},
                        compute_control= lambda x, u: np.array([0.0, 0.0]),
                        color='red')
    cars.append(car)

    world = simulator_world(lanes=lanes, cars=cars)

    sim = simulator()
    sim.init_simulator(world=world, task_name='Constant Speed')
    sim.run()
    sim.exit_simulation()

constant_speed_car()