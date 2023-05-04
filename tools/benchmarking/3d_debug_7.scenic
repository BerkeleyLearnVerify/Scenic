import trimesh

# Pick a workspace
workspace_region = BoxRegion(dimensions=(200,200,200))
workspace = Workspace(workspace_region)

ego = new Object in workspace,
    with visibleDistance 20,
    with width 5,
    with length 5,
    with height 5,
    facing (Range(0,360) deg, Range(0,360) deg, Range(0,360) deg),
    with viewAngles (120 deg, 90 deg),
    with name "obj_0"

obj_1 = new Object visible,
    with width 5,
    with length 5,
    with height 5,
    facing (Range(0,360) deg, Range(0,360) deg, Range(0,360) deg),
    with viewAngles (120 deg, 90 deg),
    with name "obj_1"

obj_2 = new Object visible from obj_1,
    with width 5,
    with length 5,
    with height 5,
    facing (Range(0,360) deg, Range(0,360) deg, Range(0,360) deg),
    with viewAngles (120 deg, 90 deg),
    with name "obj_2"

obj_3 = new Object not visible from obj_2,
    with width 5,
    with length 5,
    with height 5,
    facing (Range(0,360) deg, Range(0,360) deg, Range(0,360) deg),
    with viewAngles (120 deg, 90 deg),
    with name "obj_3"

test_point = new Point visible from obj_2

obj_4 = new Object at test_point,
    with shape MeshShape(trimesh.creation.cone(radius=0.5, height=1)),
    with width 5,
    with length 5,
    with height 5,
    facing (Range(0,360) deg, Range(0,360) deg, Range(0,360) deg),
    with viewAngles (120 deg, 90 deg),
    with occluding False

require (obj_2 can see test_point)
