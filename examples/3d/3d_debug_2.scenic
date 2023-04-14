import trimesh

# Pick a workspace
workspace_region = BoxRegion(dimensions=(25,25,25))
workspace = Workspace(workspace_region)

# Place an ego object at the origin
ego = new Object at Vector(0,0,0),
        with width 1,
        with length 1,
        with height 1

# Place many small boxes centered on the surface of a sphere
sample_space = MeshSurfaceRegion(trimesh.creation.icosphere(radius=10))

for i in range(40):
    new Object in sample_space,
        with width 0.5,
        with length 0.5,
        with height 0.5
