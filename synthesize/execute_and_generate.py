import scenic
from scenic.simulators.newtonian import NewtonianSimulator
import os
import gym

def execute_and_collect_data(path_to_scenario):
    state_action_tuples = []
    # generate data from the scenario itself.
    scenario = scenic.scenarioFromFile(path_to_scenario)
    scene, numIterations = scenario.generate()
    simulator = NewtonianSimulator()
    simulation = simulator.simulate(scene, maxSteps=100)
    if simulation:
        result = simulation.result
        for idx in range(len(result.actions)):
            # collect the (state, action) sequences for behavior cloning
            # actions are (throttle, brake, steering)
            # TODO: Ensure that the first item in the odict is always ego
            ego_action = simulation.ego_actions[idx]
            current_state = parse_state(result.trajectory[idx])
            state_action_tuples.append((current_state, ego_action))

    else:
        print("Simulation Failed! No scenario executed or data collected.")
    breakpoint()
    return state_action_tuples

def parse_state(drivingstate):
    '''
    Returns a vector (list) of size 4 * num_cars, with ego information 
    at the beginning.
    '''
    state = []
    for indiv_car in drivingstate:
        state.extend([indiv_car['position'][0], indiv_car['position'][1], 
                      indiv_car['heading'], indiv_car['speed']])
    return state


def parse_actionset(car_object):
    throttle = 0 # throttle and steering are our two actions
    steering = car_object.steer
    if car_object.hand_brake != 0:
        throttle = -1.0
    elif car_object.brake != 0:
        throttle = -1.0 * car_object.brake
    else:
        throttle = car_object.throttle

    return [throttle, steering]

def create_gym_environment(path_to_scenario):
    scenario = scenic.scenarioFromFile(path_to_scenario)
    scene, numIterations = scenario.generate()
    simulator = NewtonianSimulator()
    gymulation = simulator.createGymSimulation(scene)
    breakpoint()
    return gymulation

execute_and_collect_data(os.getcwd() + "/../examples/newtonian/overtake.scenic")
#create_gym_environment(os.getcwd() + "/../examples/newtonian/suddenstop.scenic")