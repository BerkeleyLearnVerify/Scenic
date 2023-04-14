import trimesh

# Pick a workspace
workspace_region = RectangularRegion(0 @ 0, 0, 40, 40)
workspace = Workspace(workspace_region)

forward_sample_space = BoxRegion(dimensions=(40,14,20), position=(0,12.5,10))
full_sample_space = BoxRegion(dimensions=(40,40,40), position=(0,0,0))

# Create an object at the origin who's vision cone should extend exactly
# to the edges of the workspace.
ego = new Object with visibleDistance 30,
    at (0,0,1),
    with width 5,
    with length 5,
    with height 5,
    with pitch 45 deg,
    with viewAngles (340 deg, 60 deg),
    with rayDensity 5

# Load chair mesh from file
with open(localPath("chair.obj"), "r") as mesh_file:
    mesh = trimesh.load(mesh_file, file_type="obj")

# Load chair shape at 1/10th of the original size
chair_shape = MeshShape(mesh,initial_rotation=(0,90 deg,0))

new Object in forward_sample_space,
    with width 2,
    with height 2,
    with length 2,
    with requireVisible True,
    with name "seeingnew Object"

new Object at (0,5,4),
    with width 10,
    with length 0.5,
    with height 6,
    with name "wall"
