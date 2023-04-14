import trimesh

# Pick a workspace
workspace_region = RectangularRegion(0 @ 0, 0, 40, 40)
workspace = Workspace(workspace_region)

sample_space = BoxRegion(dimensions=(30,30,30), position=(0,0,15))

ego = new Object with visibleDistance 20,
    with width 5,
    with length 5,
    with height 5,
    with viewAngles (360 deg, 180 deg)

new Object in sample_space,
    with width 1,
    with length 1,
    with height 1,
    facing (Range(0,360) deg, Range(0,360) deg, Range(0,360) deg),
    with visibleDistance 5,
    with viewAngles (360 deg, 90 deg),
    with requireVisible True

new Object in sample_space,
    with width 1,
    with length 1,
    with height 1,
    facing (Range(0,360) deg, Range(0,360) deg, Range(0,360) deg),
    with viewAngles (180 deg, 180 deg),
    with requireVisible True,
    with cameraOffset (0,0,0.5)

new Object in sample_space,
    with width 1,
    with length 1,
    with height 1,
    facing (Range(0,360) deg, Range(0,360) deg, Range(0,360) deg),
    with visibleDistance 5,
    with viewAngles (180 deg, 90 deg),
    with requireVisible True

new Object in sample_space,
    with width 1,
    with length 1,
    with height 1,
    facing (Range(0,360) deg, Range(0,360) deg, Range(0,360) deg),
    with visibleDistance 5,
    with viewAngles (200 deg, 180 deg),
    with requireVisible True

new Object in sample_space,
    with width 1,
    with length 1,
    with height 1,
    facing (Range(0,360) deg, Range(0,360) deg, Range(0,360) deg),
    with visibleDistance 5,
    with viewAngles (20 deg, 180 deg),
    with requireVisible True

new Object in sample_space,
    with width 1,
    with length 1,
    with height 1,
    facing (Range(0,360) deg, Range(0,360) deg, Range(0,360) deg),
    with visibleDistance 5,
    with viewAngles (90 deg, 90 deg),
    with requireVisible True

new Object in sample_space,
    with width 1,
    with length 1,
    with height 1,
    facing (Range(0,360) deg, Range(0,360) deg, Range(0,360) deg),
    with visibleDistance 5,
    with viewAngles (200 deg, 40 deg),
    with requireVisible True
