import trimesh

# Pick a workspace
workspace_region = BoxRegion(dimensions=(25,25,10))
workspace = Workspace(workspace_region)

# Place an ego object at the origin
ego = new Object at Vector(0,0,0),
        with width 1,
        with length 1,
        with height 1

# Place many small boxes centered on a complex surface
with open(localPath("chair.obj"), "r") as mesh_file:
    mesh = trimesh.load(mesh_file, file_type="obj")

sample_space = MeshSurfaceRegion(mesh, dimensions=(20,20,20))

new Object in sample_space
