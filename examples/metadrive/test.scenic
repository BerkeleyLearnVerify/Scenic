param map = localPath('../../assets/maps/CARLA/Town01.xodr')
param sumo_map = localPath('../../assets/maps/CARLA/Town01.net.xml')
model scenic.simulators.metadrive.model

# testing positive y axis
#ego = new Car
'''
Test results: fail? (car always faces north? orientation matches)
Initial Scenic heading: 0.8272197596083797
Initial MetaDrive heading: 0.8272197793657803

Initial Scenic heading: 1.5709181795800777
Initial MetaDrive heading: 1.5709181795810303

Initial Scenic heading: -0.00013054984817717497
Initial MetaDrive heading: -0.0001304948934137684
'''

# ego = new Car facing 0 deg
'''
test results: pass (car always faces north)
Initial Scenic heading: 2.7105054312137617e-20
Initial MetaDrive heading: 0.0
  
Initial Scenic heading: -0.0
Initial MetaDrive heading: 0.0
 
Initial Scenic heading: -0.0
Initial MetaDrive heading: 0.0
'''

ego = new Car facing 0 deg
'''
Test results: matching in scenic and metadrive (East) car facing north
Initial Scenic heading: 1.5707963267948966
Initial MetaDrive heading: 1.5707963267948966
  
Initial Scenic heading: 1.5707963267948966
Initial MetaDrive heading: 1.5707963267948966
  
Initial Scenic heading: 1.5707963267948966
Initial MetaDrive heading: 1.5707963267948966
'''