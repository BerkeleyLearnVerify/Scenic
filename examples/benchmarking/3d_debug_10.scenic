floor = RectangularRegion(0@0, 0, 10, 10)

workspace = Workspace(floor)

ego = new Object in floor, with height Range(2,3),
    with shape Uniform(ConeShape(dimensions=(0.5,1,1)), BoxShape(), CylinderShape(), SpheroidShape())
