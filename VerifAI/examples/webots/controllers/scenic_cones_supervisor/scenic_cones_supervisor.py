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
class scenic_cones(webots_task):
    def __init__(self, N_SIM_STEPS, supervisor):
        super().__init__(N_SIM_STEPS, supervisor)

    def use_sample(self, sample):
        print('Sample recieved')
        print(sample)
        cone_id = 0
        for obj in sample.objects:
            if obj.type == 'ToyotaPrius':
                object = self.supervisor.getFromDef('EgoCar')
            if obj.type == 'SmallCar':
                object = self.supervisor.getFromDef('BrokenCar')
            if obj.type == 'TrafficCone':
                object = self.supervisor.getFromDef('TrafficCone'+str(cone_id))
                cone_id +=1
            obj_pos = object.getField('translation').getSFVec3f()
            pos = obj.position
            object.getField('translation').setSFVec3f([pos[0], obj_pos[1], pos[1]])
            rot = [0, 1, 0, -obj.heading]
            object.getField('rotation').setSFRotation(rot)



    def run_task(self, sample):
        self.use_sample(sample)
        for i in range(self.N_SIM_STEPS):
            self.supervisor.step(1)

        return



PORT = 8888
BUFSIZE = 4096
N_SIM_STEPS = 20
supervisor = Supervisor()
simulation_data = DotMap()
simulation_data.port = PORT
simulation_data.bufsize = BUFSIZE
simulation_data.task = scenic_cones(N_SIM_STEPS=N_SIM_STEPS, supervisor=supervisor)
client_task = ClientWebots(simulation_data)
if not client_task.run_client():
    print("End of scene generation")
    supervisor.simulationQuit(True)
