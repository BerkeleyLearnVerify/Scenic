workspace = Workspace(RectangularRegion((0, 0), 0, 10, 10))

ego = new Object in workspace,
    facing Range(0, 360) deg,
    with visibleDistance 4,
    with viewAngles (90 deg, 90 deg),
    with showVisibleRegion True

target = new Object ahead of ego by (0, Range(2, 8)),
    with positionStdDev (0.2, 0.2, 0.2),
    with color (0, 0, 1),
    with requireVisible True

mutate target
