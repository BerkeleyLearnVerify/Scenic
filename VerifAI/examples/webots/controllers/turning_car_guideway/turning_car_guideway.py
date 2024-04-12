"""turning_car_guideway controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, LED, DistanceSensor
from vehicle import Driver
from scipy import linalg
import numpy as np
from math import atan2
from math import pi
import pickle
import os

curr_dir = os.getcwd()
par_dir = curr_dir[:curr_dir.rfind('/')]


def LQR(v_target, wheelbase, Q, R):
    # print(v_target,wheelbase,Q,R)
    A= np.matrix([[0, v_target*(5./18.)], [0, 0]])
    B = np.matrix([[0], [(v_target/wheelbase)*(5./18.)]])
    V = np.matrix(linalg.solve_continuous_are(A, B, Q, R))
    K = np.matrix(linalg.inv(R) * (B.T * V))
    return K

# create the Robot instance.
driver = Driver()

camera = driver.getCamera('camera')
camera.enable(1)

init_speed = 1 # must be bigger than 0
speed = init_speed
wheelbase = 2.8
cruising_speed = 30
car_length = 3.0

driver.setSteeringAngle(0.0)
driver.setCruisingSpeed(30)

while driver.step() != -1:
    driver.setCruisingSpeed(30)

    # get speed
    speed = max(init_speed, driver.getCurrentSpeed())

    # get the LQR feedback law for current speed
    K = LQR(speed,wheelbase,np.array([[1, 0], [0, 3]]), np.array([[50]]))
   
    # get data from the supervisor
    while True:
        try:
            with open(par_dir+'/scenic_intersection_supervisor/data_turning.pickle', "rb") as f:
                turning_data = pickle.load(f)
                th_s, x_s, y_s = -turning_data.turning.theta, \
                                 turning_data.turning.x, turning_data.turning.y
                x1, y1, x2, y2 = turning_data.line.x1, turning_data.line.y1, \
                                 turning_data.line.x2, turning_data.line.y2
                break
        except:
            pass

    # segment orientation
    deltaTh = atan2(x2-x1, y2-y1)

    # relative orientation
    th_n = th_s + deltaTh
    # change of basis to use it for LQR
    th_n = (th_n + 3*pi) % (2*pi) - pi

    # deviation from the line
    x_n = (x2-x1)*y_s - (y2-y1)*x_s + y2*x1 - y1*x2
    x_n /= np.linalg.norm(np.array([x1, y1]) - np.array([x2, y2]))


    # calculate steering
    u = -K * np.matrix([[x_n], [th_n]])
    u = np.double(u)
    u = min(u, np.pi/4.)
    u = max(u, -np.pi/4.)
    driver.setSteeringAngle(u)
