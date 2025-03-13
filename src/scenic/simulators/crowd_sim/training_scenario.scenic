from scenic.simulators.crowd_sim.model import *
from scenic.simulators.crowd_sim.simulator import CrowdSimSimulator

param verifaiSamplerType = 'halton'

param x = VerifaiRange(-8, 8)
param y = VerifaiRange(-8, 8)
param gx = VerifaiRange(-8, 8)
param gy = VerifaiRange(-8, 8)

param hx1 = VerifaiRange(-8, 8)
param hy1 = VerifaiRange(-8, 8)

param hx2 = VerifaiRange(-8, 8)
param hy2 = VerifaiRange(-8, 8)

param hx3 = VerifaiRange(-8, 8)
param hy3 = VerifaiRange(-8, 8)

simulator CrowdSimSimulator()


ego = new Robot at (globalParameters.x, globalParameters.y, 0), with yaw 0 deg, 
                    with goal (globalParameters.gx, globalParameters.gy, 0)

# ego = new Robot at (Range(-3, 3), Range(-3, 0), 0), with yaw 0 deg, 
                    # with goal (globalParameters.gx, globalParameters.gy, 0)

human1 = new Human at (globalParameters.hx1, globalParameters.hy1, 0), 
                    with name "human1"
# human1 = new Human at (Range(-3, 3), Range(1.5, 4), 0), 
                    # with name "human1"

human2 = new Human at (globalParameters.hx2, globalParameters.hy2, 0), with name "human2"

human3 = new Human at (globalParameters.hx3, globalParameters.hy3, 0), with name "human3"

terminate when ego.collision
