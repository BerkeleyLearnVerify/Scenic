from verifai.simulators.webots.webots_task import webots_task
from verifai.simulators.webots.client_webots import ClientWebots
from math import sin
from math import cos
import numpy as np
from math import atan2
from collections import namedtuple
import os
from dotmap import DotMap
import pickle
from shapely.geometry import Point, Polygon

try:
    from controller import Supervisor
except ModuleNotFoundError:
    import sys
    sys.exit('This functionality requires webots to be installed')


# get distance from x, y to point i
def getDist(data, xy):
    return np.linalg.norm(np.array(data) - np.array(xy), axis=1)


# get two nearest points
Line = namedtuple('Line', ['x1', 'y1', 'x2', 'y2'])


def getLine(x, y, data):
    dists = getDist(data=data, xy=[x, y])
    dist_pos = np.argpartition(dists, 2)
    i, j = dist_pos[0], dist_pos[1]
    x1, y1 = data[min(i, j)][0], data[min(i, j)][1]
    x2, y2 = data[max(i, j)][0], data[max(i, j)][1]
    return Line(x1=x1, y1=y1, x2=x2, y2=y2)

curr_dir = os.getcwd()
par_dir = curr_dir


# Defining the task as a webots task
class scenic_intersection(webots_task):
    def __init__(self, N_SIM_STEPS, supervisor):
        super().__init__(N_SIM_STEPS, supervisor)

    def use_sample(self, sample):
        print('Sample recieved')
        print(sample)
        self.data = sample.params.turnWaypoints
        car_id = 0
        for obj in sample.objects:
            if obj.webotsType == 'TurningCar':
                object = self.supervisor.getFromDef('TurningCar')
                turning_car = object
                offset = -20
            if obj.webotsType == 'Ego':
                object = self.supervisor.getFromDef('EgoCar')
                ego_car = object
                offset = 30
            if obj.webotsType == 'ToyotaPrius':
                object = self.supervisor.getFromDef('waiting_car'+str(car_id+3))
                car_id +=1
                offset = 0
            obj_pos = object.getField('translation').getSFVec3f()
            pos = obj.position
            object.getField('translation').setSFVec3f([pos[0], obj_pos[1], pos[1]+offset])
            rot = [0, 1, 0, -obj.heading]
            object.getField('rotation').setSFRotation(rot)
        return ego_car, turning_car


    def run_task(self, sample):
        ego_car, turning_car = self.use_sample(sample)

        car_length = 2.995
        intersection = [(-28, 25), (-28, 17), (-7, 17), (-7, 25)]
        destination = max([y for (x, y) in intersection])
        intersection_safe = 1
        for _ in range(self.N_SIM_STEPS):
            self.supervisor.step(1)
            # get car position
            turning_x = turning_car.getPosition()[0]
            turning_y = turning_car.getPosition()[2]
            turning_th = atan2(turning_car.getOrientation()[2], turning_car.getOrientation()[0])

            # get position of front of the car
            turning_y = turning_y + car_length * cos(turning_th)
            turning_x = turning_x + car_length * sin(turning_th)

            # find nearest segment
            line = getLine(x=turning_x, y=turning_y, data=self.data)

            # send data to the controller
            write_data = DotMap()
            write_data.turning.theta = turning_th
            write_data.turning.x = turning_x
            write_data.turning.y = turning_y
            write_data.line.x1 = line.x1
            write_data.line.y1 = line.y1
            write_data.line.x2 = line.x2
            write_data.line.y2 = line.y2

            pickle.dump(write_data, open(par_dir + '/data_turning.pickle', 'wb'))

            ego_x = ego_car.getPosition()[0]
            ego_y = ego_car.getPosition()[2]
            ego_th = atan2(ego_car.getOrientation()[2], ego_car.getOrientation()[0])
            ego_y = ego_y + car_length * cos(ego_th)
            ego_x = ego_x + car_length * sin(ego_th)

            intersection_buffer = 2.5 # This decides how far you are from the intersection to provide the warning
                                      # If 0 then you provide warning when you just enter the intersection
            intersection_polygon = Polygon(intersection)
            intersection_polygon = intersection_polygon.buffer(intersection_buffer + 0.5)
            intersection_safe = not Point(turning_x, turning_y).within(intersection_polygon)

            write_data = DotMap()
            write_data.turning.theta = turning_th
            write_data.turning.x = turning_x
            write_data.turning.y = turning_y
            write_data.braking_info.safe = intersection_safe
            write_data.braking_info.dist = ego_y - destination


            pickle.dump(write_data, open(par_dir + '/data_ego.pickle', 'wb'))

        return



PORT = 8888
BUFSIZE = 4096
N_SIM_STEPS = 300
supervisor = Supervisor()
simulation_data = DotMap()
simulation_data.port = PORT
simulation_data.bufsize = BUFSIZE
simulation_data.task = scenic_intersection(N_SIM_STEPS=N_SIM_STEPS, supervisor=supervisor)
client_task = ClientWebots(simulation_data)
if not client_task.run_client():
    print("End of accident scenario generation")
    supervisor.simulationQuit(True)
