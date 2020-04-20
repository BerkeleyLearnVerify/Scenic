
from scenic.simulators.webots.guideways.intersection import setLocalIntersection
setLocalIntersection(__file__, 'McClintock_DonCarlos_Tempe.json')

from scenic.simulators.webots.guideways.model import *

ego = Car in intersection.mainGuideway.region
