from vehicle import Driver
import os
import pickle

curr_dir = os.getcwd()
par_dir = curr_dir[:curr_dir.rfind('/')]

driver = Driver()

driver.setSteeringAngle(0.0)
driver.setGear(1)
driver.setThrottle(1)
driver.setCruisingSpeed(100)

camera = driver.getCamera('camera')
camera.enable(1)

brake_coeff = 9.0
counter = 0
braking = False
braking_threshold = 0.3
friction_acc = 0.5

ignore_smart_intersection = True # Set this to True if you want to ignore smart intersection
while driver.step() != -1:
    counter = counter + 1

    speed = driver.getCurrentSpeed() * (5. / 18.)

    while True:
        try:
            with open(par_dir + '/scenic_intersection_supervisor/data_ego.pickle', "rb") as f:
                turning_data = pickle.load(f)
                th_s, x_s, y_s = -turning_data.turning.theta, \
                                 turning_data.turning.x, turning_data.turning.y
                intersection_safe, dist = turning_data.braking_info.safe, \
                                          turning_data.braking_info.dist
                break
        except:
            pass

    if str(speed) == 'nan' or intersection_safe or ignore_smart_intersection:
        continue
    if not intersection_safe:
        print("Human car at intersection")
    if not intersection_safe:
        # If very close to intersection safer to just go ahead.
        intersection_safe = intersection_safe or dist < 10
        if intersection_safe:
            continue
        print("Uber car slow down. Distance to intersection: ", dist)

    # driver.setBrakeIntensity(1.0)
    a = (speed ** 2) / (2 * dist)
    if a >= 0:
        b = max(min((a - friction_acc) / brake_coeff, 1), 0.001)
        # b = min(a/brake_coef, 1)
    else:
        b = 1
    if b > braking_threshold:
        braking = True
    if braking:
        driver.setThrottle(0)
        driver.setBrakeIntensity(b)




