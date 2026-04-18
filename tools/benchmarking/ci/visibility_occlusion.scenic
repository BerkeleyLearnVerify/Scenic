workspace = Workspace(BoxRegion(dimensions=(10, 10, 10)))
innerRegion = BoxRegion(dimensions=(7, 7, 7))

region = BoxRegion().difference(BoxRegion(dimensions=(0.5, 0.5, 0.5)))
shape = MeshShape(region.mesh)

obstacle = new Object contained in innerRegion,
    with shape shape,
    with width 2, with height 2, with length 2,
    with positionStdDev (1, 1, 1),
    with name "obstacle"

ego = new Object contained in workspace,
    facing directly toward obstacle,
    with visibleDistance 20,
    with viewAngles (90 deg, 90 deg),
    with showVisibleRegion True,
    with name "ego"

target = new Object beyond obstacle by 2,
    with positionStdDev (0.2, 0.2, 0.2),
    with color (0, 0, 1),
    with requireVisible True,
    with name "target"

mutate obstacle, target
