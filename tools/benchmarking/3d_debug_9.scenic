import trimesh

# Pick a workspace
workspace_region = RectangularRegion(0 @ 0, 0, 40.1, 40.1)
workspace = Workspace(workspace_region)

ego = new Object with visibleDistance 20,
    with viewAngles (120 deg, 0.1)

new Object visible from ego

on_object = new Object on workspace

require ego can see on_object
