'''
To run this file using the MetaDrive simulator:
    scenic examples/driving/arya.scenic --2d --model scenic.simulators.metadrive.model --simulate

To run this file using the Carla simulator:
    scenic examples/driving/arya.scenic --2d --model scenic.simulators.carla.model --simulate


TODO:
    Make some kinda waypoints that the car can follow 
    make the car follow the waypoints using pure pursuit.
'''

param map = localPath('../../assets/maps/CARLA/Town05.xodr')
param carla_map = 'Town05'
param time_step = 1.0/10

model scenic.domains.driving.model

behavior PullIntoRoad():
    while (distance from self to ego) > 15:
        wait
    do FollowLaneBehavior(laneToFollow=ego.lane)

ego = new Car with behavior DriveAvoidingCollisions(avoidance_threshold=5, pure_pursuit=True, plot=True)

rightCurb = ego.laneGroup.curb
spot = new OrientedPoint on visible rightCurb
badAngle = Uniform(1.0, -1.0) * Range(10, 20) deg
#parkedCar = new Car left of spot by 0.5,
                #facing badAngle relative to roadDirection,
                #with behavior PullIntoRoad

#require (distance to parkedCar) > 20

monitor StopAfterInteraction():
    for i in range(50):
        wait
    while ego.speed > 2:
        wait
    for i in range(50):
        wait
    terminate
require monitor StopAfterInteraction()
terminate after 15 seconds   # in case ego never breaks

"""
['_3DClass', '__class__', '__delattr__', '__dict__', '__dir__', '__doc__', '__eq__', 
'__format__', '__ge__', '__getattr__', '__getattribute__', '__getnewargs_ex__', '__getstate__', 
'__gt__', '__hash__', '__init__', '__init_subclass__', '__le__', '__lt__', '__module__', 
'__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', 
'__str__', '__subclasshook__', '__weakref__', '_allProperties', '_boundingPolygon', 
'_cache_clearers', '_canSee2D', '_clearCaches', '_collect_action', '_conditioned', 
'_constProps', '_control', '_copyWith', '_corners2D', '_crossing', '_defaults', 
'_dependencies', '_dynamicFinalProperties', '_dynamicProperties', '_dynamicProxy', '_element', 
'_finalProperties', '_hasStaticBounds', '_intersection', '_isConvex', '_isLazy', '_isPlanarBox', 
'_lane', '_laneGroup', '_laneSection', '_needsLazyEval', '_needsSampling', '_nonObservingEntity', 
'_observingEntity', '_override', '_parentScenario', '_prepareSpecifiers', '_propertiesSet', 
'_props_transformed', '_recomputeDynamicFinals', '_register', '_relations', '_requiredProperties', 
'_reset_control', '_resolveSpecifiers', '_revert', '_road', '_sampleParent', '_scaledShape', 
'_scenic_properties', '_simulatorProvidedProperties', '_specify', '_with', '_withProperties', 
'_withSpecifiers', 'allowCollisions', 'angularSpeed', 'angularVelocity', 'back', 'backLeft', 
'backRight', 'backSurface', 'baseOffset', 'behavior', 'bottom', 'bottomBackLeft', 'bottomBackRight', 
'bottomFrontLeft', 'bottomFrontRight', 'bottomSurface', 'boundingBox', 'cameraOffset', 'canSee', 
'color', 'conditionTo', 'contactTolerance', 'containsPoint', 'corners', 'crossing', 'deserializeValue', 
'distancePast', 'distanceTo', 'distanceToClosest', 'dumpAsScenicCode', 'element', 'elevation', 
'evaluateIn', 'evaluateInner', 'front', 'frontLeft', 'frontRight', 'frontSurface', 'getContextValues', 
'heading', 'headingStdDev', 'height', 'hh', 'hl', 'hw', 'inradius', 'intersection', 'intersects', 
'isCar', 'isVehicle', 'lane', 'laneGroup', 'laneSection', 'lastActions', 'left', 'leftSurface', 
'length', 'makeContext', 'metaDriveActor', 'mutationScale', 'mutator', 'occluding', 'occupiedSpace', 
'onDirection', 'onSurface', 'oppositeLaneGroup', 'orientation', 'orientationStdDev', 'parentOrientation', 
'pitch', 'planarInradius', 'position', 'positionStdDev', 'properties', 'radius', 'regionContainedIn', 
'relativePosition', 'relativize', 'render', 'requireVisible', 'right', 'rightSurface', 'road', 
'roadDeviation', 'roll', 'sample', 'sampleAll', 'sampleGiven', 'serializeValue', 'setBraking', 
'setHandbrake', 'setPosition', 'setReverse', 'setSteering', 'setThrottle', 'setVelocity', 'shape', 
'show2D', 'show3D', 'showVisibleRegion', 'sideComponentThresholds', 'speed', 'startDynamicSimulation', 
'surface', 'toHeading', 'toOrientation', 'toVector', 'top', 'topBackLeft', 'topBackRight', 
'topFrontLeft', 'topFrontRight', 'topSurface', 'velocity', 'viewAngle', 'viewAngles', 'viewRayCount', 
'viewRayDensity', 'viewRayDistanceScaling', 'visibleDistance', 'visibleRegion', 'width', 'yaw']
"""