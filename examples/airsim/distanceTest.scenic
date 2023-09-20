model scenic.simulators.airsim.model

param airsimWorldInfoPth = "path here"

new Drone at (10,10,10)
new Drone at (20,20,20)

new StaticObj at (0,10,0),
    with width 2,
    with assetName "Cone",
    with name "fun"
